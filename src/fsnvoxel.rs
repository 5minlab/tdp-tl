use super::{BoundingBox, Model, Voxel, VoxelIdx, UNIT};

use super::cell::*;
use super::chunkedvoxel::ChunkedBase;
use anyhow::Result;
use fast_surface_nets::ndshape::{ConstShape, ConstShape3u32};
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};
use std::collections::*;
use std::rc::Rc;

// A 16^3 chunk with 1-voxel boundary padding.
const PADDED_SIZE: u32 = CELL_SIZE as u32 + 2;
type ChunkShape = ConstShape3u32<PADDED_SIZE, PADDED_SIZE, PADDED_SIZE>;

fn write_model<W: std::io::Write>(model: &Model, mut writer: W) -> Result<()> {
    use byteorder::{LittleEndian, WriteBytesExt};

    writer.write_u64::<LittleEndian>(model.id)?;
    writer.write_u32::<LittleEndian>(model.raw_vertices.len() as u32 * 3)?;
    for [x, y, z] in &model.raw_vertices {
        writer.write_f32::<LittleEndian>(*x)?;
        writer.write_f32::<LittleEndian>(*y)?;
        writer.write_f32::<LittleEndian>(*z)?;
    }

    writer.write_u32::<LittleEndian>(model.raw_normals.len() as u32 * 3)?;
    for [x, y, z] in &model.raw_normals {
        writer.write_f32::<LittleEndian>(*x)?;
        writer.write_f32::<LittleEndian>(*y)?;
        writer.write_f32::<LittleEndian>(*z)?;
    }

    writer.write_u32::<LittleEndian>(model.raw_triangles.len() as u32 * 3)?;
    for [i0, i1, i2] in &model.raw_triangles {
        writer.write_u32::<LittleEndian>(*i0)?;
        writer.write_u32::<LittleEndian>(*i1)?;
        writer.write_u32::<LittleEndian>(*i2)?;
    }
    Ok(())
}

#[derive(Default)]
pub struct FSNVoxel {
    base: ChunkedBase,

    dirty: HashSet<VoxelIdx>,
    model_cache: HashMap<u64, Rc<Model>>,
}

impl FSNVoxel {
    fn setdirty(&mut self, coord_dirty: VoxelIdx) {
        if self.dirty.insert(coord_dirty) {
            let coord = coord_dirty.shift_up(CELL_SIZE_BITS);
            self.model_cache.remove(&chunk_idx(coord));
        }
    }

    pub fn write_dirty<W: std::io::Write>(&mut self, mut writer: W) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        writer.write_u32::<LittleEndian>(1)?;
        writer.write_f32::<LittleEndian>(UNIT)?;

        let dirty = std::mem::take(&mut self.dirty);
        let mut indices = Vec::new();
        for coord_dirty in dirty.iter() {
            let coord = coord_dirty.shift_up(CELL_SIZE_BITS);
            let idx = chunk_idx(coord);
            if !self.base.chunks.contains_key(&idx) {
                continue;
            }
            indices.push(idx);
        }

        writer.write_u32::<LittleEndian>(indices.len() as u32)?;
        for idx in indices.iter() {
            let idx = *idx;
            let model = self.build_model(idx);
            self.model_cache.insert(idx, model.clone());
            write_model(&model, &mut writer)?;
        }

        Ok(())
    }

    fn build_model(&mut self, idx: u64) -> Rc<Model> {
        let cell = self.base.chunks.get(&idx).unwrap();
        let base = chunk_base(idx);
        let bx = base[0] as u32;
        let by = base[1] as u32;
        let bz = base[2] as u32;

        let mut sdf = [1.0; ChunkShape::USIZE];
        for i in 0u32..ChunkShape::SIZE {
            let [x, y, z] = ChunkShape::delinearize(i);
            if x == 0
                || y == 0
                || z == 0
                || x == (PADDED_SIZE - 1)
                || y == (PADDED_SIZE - 1)
                || z == (PADDED_SIZE - 1)
            {
                let worldpos = VoxelIdx::new([
                    bx as i32 + x as i32 - 1,
                    by as i32 + y as i32 - 1,
                    bz as i32 + z as i32 - 1,
                ]);
                if self.occupied(worldpos) {
                    sdf[i as usize] = -1.0;
                }
                continue;
            }
            if cell.get(x as usize - 1, y as usize - 1, z as usize - 1) {
                sdf[i as usize] = -1.0;
            }
        }

        let mut mesh = SurfaceNetsBuffer::default();
        surface_nets(&sdf, &ChunkShape {}, [0; 3], [33; 3], &mut mesh);

        let indices = if true {
            mesh.indices.clone()
        } else {
            let data: &[u8] = unsafe {
                let ptr: *const u8 = mesh.positions[0].as_ptr() as *const u8;
                let len = mesh.positions.len() * 4 * 3;
                std::slice::from_raw_parts(ptr, len)
            };

            let adapter = meshopt::utilities::VertexDataAdapter::new(data, 12, 0).unwrap();
            let mut error = 0f32;
            let indices = meshopt::simplify::simplify(
                &mesh.indices,
                &adapter,
                0,
                0.001f32,
                meshopt::simplify::SimplifyOptions::None,
                Some(&mut error),
            );
            // eprintln!("{:?} -> {:?}, error={:.03}", mesh.indices.len(), indices.len(), error);
            indices
        };

        // oldindex -> newindex
        let mut set = std::collections::HashMap::new();
        let mut new_indices = Vec::with_capacity(indices.len());
        for oi in &indices {
            let oi = *oi;
            if !set.contains_key(&oi) {
                let ni = set.len() as u32;
                set.insert(oi, ni);
                new_indices.push(oi);
            }
        }

        let mut model = Model::default();
        model.id = idx;
        for oi in &new_indices {
            let oi = *oi as usize;
            let pos = mesh.positions[oi];
            model
                .raw_vertices
                .push([pos[0] + bx as f32, pos[1] + by as f32, pos[2] + bz as f32]);
            model.raw_normals.push(mesh.normals[oi]);
        }

        let triangle_count = indices.len() / 3;
        for i in 0..triangle_count {
            let oi1 = indices[i * 3];
            let oi2 = indices[i * 3 + 1];
            let oi3 = indices[i * 3 + 2];

            let ni1 = *set.get(&oi1).unwrap();
            let ni2 = *set.get(&oi2).unwrap();
            let ni3 = *set.get(&oi3).unwrap();

            model.raw_triangles.push([ni1, ni2, ni3]);
        }

        Rc::new(model)
    }
}

impl Voxel for FSNVoxel {
    fn ranges(&self) -> usize {
        0
    }

    fn bounding_box(&self) -> &BoundingBox {
        &self.base.bb
    }

    fn occupied(&self, coord: VoxelIdx) -> bool {
        self.base.occupied(coord)
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
        let added = self.base.add(coord);
        if !added {
            return false;
        }

        let coord_dirty = coord.shift_down(CELL_SIZE_BITS);
        self.setdirty(coord_dirty);

        let [xx, yy, zz] = cell_idx(coord);
        if xx == 0 {
            self.setdirty(coord_dirty + VoxelIdx::new([-1, 0, 0]));
        } else if xx == (CELL_SIZE - 1) {
            self.setdirty(coord_dirty + VoxelIdx::new([1, 0, 0]));
        }
        if yy == 0 {
            self.setdirty(coord_dirty + VoxelIdx::new([0, -1, 0]));
        } else if yy == (CELL_SIZE - 1) {
            self.setdirty(coord_dirty + VoxelIdx::new([0, 1, 0]));
        }
        if zz == 0 {
            self.setdirty(coord_dirty + VoxelIdx::new([0, 0, -1]));
        } else if zz == (CELL_SIZE - 1) {
            self.setdirty(coord_dirty + VoxelIdx::new([0, 0, 1]));
        }

        true
    }

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut models = vec![];

        let indices = self.base.chunks.keys().cloned().collect::<Vec<_>>();

        for idx in indices {
            if let Some(model) = self.model_cache.get(&idx) {
                models.push(model.clone());
                continue;
            }
            let model = self.build_model(idx);
            self.model_cache.insert(idx, model.clone());
            models.push(model);
        }

        models
    }

    fn debug0(&mut self, filename: &str) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        let writer = std::fs::File::create(filename)?;
        let mut writer = std::io::BufWriter::new(writer);

        // type
        writer.write_u32::<LittleEndian>(1)?;
        writer.write_f32::<LittleEndian>(UNIT)?;

        let models = self.to_model();
        writer.write_u32::<LittleEndian>(models.len() as u32)?;
        for model in models {
            write_model(&model, &mut writer)?;
        }

        Ok(())
    }

    fn debug1(&mut self) -> usize {
        let mut buf = Vec::new();
        self.write_dirty(&mut buf).unwrap();
        buf.len()
        /*
        let len = self.dirty.len();
        self.dirty.clear();
        len
            */
    }
}

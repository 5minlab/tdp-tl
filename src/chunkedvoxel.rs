use super::{BoundingBox, Model, Voxel, VoxelIdx, UNIT};
use anyhow::Result;
use std::collections::BTreeSet;

use crate::cell::*;
use ahash::AHashMap;
use binary_greedy_meshing as bgm;
use std::collections::HashMap;
use std::rc::Rc;

#[derive(Default)]
pub struct ChunkedBase {
    pub chunks: AHashMap<u64, BGMCell>,
    pub bb: BoundingBox,
}

impl ChunkedBase {
    pub fn occupied(&self, coord: VoxelIdx) -> bool {
        let idx = chunk_idx(coord);
        if let Some(cell) = self.chunks.get(&idx) {
            let [x, y, z] = cell_idx(coord);
            cell.get(x, y, z)
        } else {
            false
        }
    }

    pub fn add(&mut self, coord: VoxelIdx) -> bool {
        let idx = chunk_idx(coord);
        let [x, y, z] = cell_idx(coord);

        if let Some(cell) = self.chunks.get_mut(&idx) {
            self.bb.add(coord);

            if cell.get(x, y, z) {
                return false;
            }

            cell.set(x, y, z);
            true
        } else {
            let mut cell = BGMCell::default();
            self.bb.add(coord);

            cell.set(x, y, z);
            self.chunks.insert(idx, cell);
            true
        }
    }
}

#[derive(Default)]
pub struct ChunkedVoxel {
    base: ChunkedBase,
    models: HashMap<u64, Rc<Model>>,
}

impl Voxel for ChunkedVoxel {
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
        self.base.add(coord)
    }

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut models = vec![];
        let mut voxels = [0; bgm::CS_P3];

        let mut dirty = 0;
        let mut count = 0;
        for (&idx, cell) in self.base.chunks.iter() {
            if !cell.dirty.get() {
                if let Some(model) = self.models.get(&idx) {
                    models.push(model.clone());
                }
                continue;
            }
            dirty += 1;

            let base = chunk_base(idx);

            let mut model = Model::default();
            model.offset = base;
            count += cell.to_model(&mut voxels, &mut model);
            cell.dirty.set(false);

            let model = Rc::new(model);
            self.models.insert(idx, model.clone());
            models.push(model);
        }

        eprintln!(
            "dirty: {}/{}, quad_count: {}",
            dirty,
            self.base.chunks.len(),
            count
        );

        models
    }

    fn debug(&self, filename: &str) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        let writer = std::fs::File::create(filename)?;
        let mut writer = std::io::BufWriter::new(writer);
        let mut voxels = [0; bgm::CS_P3];

        writer.write_f32::<LittleEndian>(UNIT)?;

        writer.write_u32::<LittleEndian>(self.base.chunks.len() as u32)?;
        for (&idx, cell) in self.base.chunks.iter() {
            let base = chunk_base(idx);
            writer.write_i32::<LittleEndian>(base[0])?;
            writer.write_i32::<LittleEndian>(base[1])?;
            writer.write_i32::<LittleEndian>(base[2])?;

            // cell.fill_bgm(&mut voxels, 0);
            cell.fill_bgm_solid(&mut voxels);
            let mut mesh_data = bgm::MeshData::new();
            bgm::mesh(&mut voxels, &mut mesh_data, BTreeSet::default());

            for quads in mesh_data.quads.iter() {
                writer.write_u32::<LittleEndian>(quads.len() as u32)?;
                for quad in quads.iter() {
                    writer.write_u32::<LittleEndian>(*quad as u32)?;
                }
            }
        }

        Ok(())
    }
}

use super::*;
use crate::cell::*;
use ahash::*;
use anyhow::Result;
use std::collections::BTreeSet;
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
        } else {
            let mut cell = BGMCell::default();
            self.bb.add(coord);

            cell.set(x, y, z);
            self.chunks.insert(idx, cell);
        }
        true
    }
}

fn write_cell0<W: std::io::Write>(idx: u64, cell: &BGMCell, mut writer: W) -> Result<()> {
    use byteorder::{LittleEndian, WriteBytesExt};

    let base = chunk_base(idx);

    writer.write_u64::<LittleEndian>(idx)?;
    writer.write_i32::<LittleEndian>(base[0])?;
    writer.write_i32::<LittleEndian>(base[1])?;
    writer.write_i32::<LittleEndian>(base[2])?;

    let mut voxels = [0; CS_P3];
    // cell.fill_bgm(&mut voxels, 0);
    cell.fill_bgm_solid(&mut voxels);
    let mut mesher = Mesher::new();
    let transparent = BTreeSet::new();
    mesher.mesh(&mut voxels, &transparent);

    for quads in mesher.quads.iter() {
        writer.write_u32::<LittleEndian>(quads.len() as u32)?;
        for quad in quads.iter() {
            writer.write_u32::<LittleEndian>(quad.0 as u32)?;
        }
    }

    Ok(())
}

fn write_cell<W: std::io::Write>(
    idx: u64,
    cell: &BGMCell,
    writer: W,
    ro: WriteOptions,
) -> Result<()> {
    match ro {
        WriteOptions::None => write_cell0(idx, cell, writer),
        WriteOptions::Simplify => {
            let mut cell = cell.clone();
            cell.simplify();
            write_cell0(idx, &cell, writer)
        }
    }
}

#[derive(Default)]
pub struct ChunkedVoxel {
    base: ChunkedBase,
    models: HashMap<u64, Rc<Model>>,

    ro: WriteOptions,
    dirty: HashSet<VoxelIdx>,
    model_cache: HashMap<u64, Rc<Model>>,
}

impl ChunkedVoxel {
    pub fn setdirty(&mut self, coord_dirty: VoxelIdx) {
        if self.dirty.insert(coord_dirty) {
            let coord = coord_dirty.shift_up(CELL_SIZE_BITS);
            self.model_cache.remove(&chunk_idx(coord));
        }
    }
}

impl StreamingVoxel for ChunkedVoxel {
    fn write_dirty<W: std::io::Write>(&mut self, mut writer: W) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        writer.write_u32::<LittleEndian>(2)?;

        let dirty = std::mem::take(&mut self.dirty);
        let mut indices = Vec::new();
        for coord_dirty in dirty.iter() {
            let coord = coord_dirty.shift_up(CELL_SIZE_BITS);
            let idx = chunk_idx(coord);
            indices.push(idx);
        }

        writer.write_u32::<LittleEndian>(indices.len() as u32)?;
        for idx in indices.iter() {
            let idx = *idx;
            write_cell(
                idx,
                self.base.chunks.get(&idx).unwrap(),
                &mut writer,
                self.ro,
            )?;
        }

        Ok(())
    }
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
        let added = self.base.add(coord);
        if !added {
            return false;
        }

        let coord_dirty = coord.shift_down(CELL_SIZE_BITS);
        self.setdirty(coord_dirty);
        true
    }

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut models = vec![];
        let mut voxels = [0; CS_P3];

        for (&idx, cell) in self.base.chunks.iter() {
            if let Some(model) = self.model_cache.get(&idx) {
                models.push(model.clone());
                continue;
            }

            let base = chunk_base(idx);

            let mut model = Model::default();
            model.offset = base;
            cell.to_model(&mut voxels, &mut model);

            let model = Rc::new(model);
            self.models.insert(idx, model.clone());
            models.push(model);
        }

        models
    }

    fn write_binary<W: std::io::Write>(&mut self, mut writer: W) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        // type
        writer.write_u32::<LittleEndian>(2)?;

        writer.write_u32::<LittleEndian>(self.base.chunks.len() as u32)?;
        for (&idx, cell) in self.base.chunks.iter() {
            write_cell(idx, cell, &mut writer, self.ro)?;
        }

        Ok(())
    }

    fn debug1(&mut self) -> usize {
        /*
        let mut out = vec![];
        self.write_dirty(&mut out).unwrap();
        // eprintln!("{} bytes written", out.len());

        out.len()
        */
        0
    }

    fn set_options(&mut self, options: WriteOptions) {
        if options != self.ro {
            self.ro = options;
            self.model_cache.clear();
            self.dirty.clear();
        }
    }
}

use super::{BoundingBox, Model, Voxel, VoxelIdx};

use crate::cell::*;
use ahash::AHashMap;
use std::collections::HashMap;
use std::rc::Rc;

use binary_greedy_meshing as bgm;
#[derive(Default)]
pub struct ChunkedVoxel {
    chunks: AHashMap<u64, BGMCell>,
    models: HashMap<u64, Rc<Model>>,
    bb: BoundingBox,
}

impl Voxel for ChunkedVoxel {
    fn ranges(&self) -> usize {
        0
    }

    fn bounding_box(&self) -> &BoundingBox {
        &self.bb
    }

    fn occupied(&self, coord: VoxelIdx) -> bool {
        let idx = chunk_idx(coord);
        if let Some(cell) = self.chunks.get(&idx) {
            let [x, y, z] = cell_idx(coord);
            cell.get(x, y, z)
        } else {
            false
        }
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
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

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut models = vec![];
        let mut voxels = [0; bgm::CS_P3];

        let mut dirty = 0;
        let mut count = 0;
        for (&idx, cell) in self.chunks.iter() {
            if !cell.dirty.get() {
                if let Some(model) = self.models.get(&idx) {
                    models.push(model.clone());
                }
                continue;
            }
            dirty += 1;

            let base = chunk_base(idx);

            let mut model = Model::default();
            count += cell.to_model(base, &mut voxels, &mut model);
            cell.dirty.set(false);
            let model = Rc::new(model);
            self.models.insert(idx, model.clone());
            models.push(model);
        }

        eprintln!(
            "dirty: {}/{}, quad_count: {}",
            dirty,
            self.chunks.len(),
            count
        );

        models
    }
}

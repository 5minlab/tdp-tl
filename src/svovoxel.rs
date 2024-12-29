use std::num::NonZeroU32;

use svo_rs::*;
use super::{BoundingBox, Model, Voxel, VoxelIdx};

const SIZE: u32 = 65536;
const OFFSET: u32 = SIZE / 2;

fn to_voxel_idx(coord: VoxelIdx) -> Option<[u32; 3]> {
    let size = SIZE as i32;
    let offset = OFFSET as i32;

    let idx = coord.idx;
    let x = idx[0] + offset;
    let y = idx[1] + offset;
    let z = idx[2] + offset;
    if x < 0 || x >= size || y < 0 || y >= size || z < 0 || z >= size {
        None
    } else {
        Some([x as u32, y as u32, z as u32])
    }
}

pub struct SVOVoxel {
    inner: Octree<u8>,
    bb: BoundingBox,
}

impl std::default::Default for SVOVoxel {
    fn default() -> Self {
        Self {
            inner: Octree::new(NonZeroU32::new(SIZE).unwrap()).unwrap(),
            bb: BoundingBox::default(),
        }
    }
}

impl Voxel for SVOVoxel {
    fn ranges(&self) -> usize {
        0
    }

    fn bounding_box(&self) -> &BoundingBox {
        &self.bb
    }

    fn occupied(&self, coord: VoxelIdx) -> bool {
        if let Some(coord1) = to_voxel_idx(coord) {
            self.inner.get(coord1).is_some()
        } else {
            false
        }
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
        if let Some(coord1) = to_voxel_idx(coord) {
            self.bb.add(coord);
            self.inner.insert(coord1, 1).is_ok()
        } else {
            false
        }
    }

    fn to_model(&self) -> Model {
        Model::default()
    }
}


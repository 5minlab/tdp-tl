use log::*;
use std::num::NonZeroU32;

use super::{BoundingBox, Model, Voxel, VoxelIdx};
use svo_rs::*;

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

fn from_voxel_idx(coord: [u32; 3]) -> VoxelIdx {
    let offset = OFFSET as i32;

    let x = coord[0] as i32 - offset;
    let y = coord[1] as i32 - offset;
    let z = coord[2] as i32 - offset;

    VoxelIdx::new([x, y, z])
}

pub struct SVOVoxel {
    inner: Octree<bool>,
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

fn visit_quad2(node: &Node<bool>, depth: u32, max_depth: u32, model: &mut Model) -> usize {
    let mut count = 0;
    match node.ty {
        NodeType::Leaf(_) => {
            let b = node.bounds;
            let idx = from_voxel_idx([b[0].x, b[0].y, b[0].z]);
            let size = b[1].x - b[0].x;
            model.add_cube_size(idx, size as i32);
            count += 1;
        }
        NodeType::Internal => {
            for child in node.children.iter() {
                if let Some(ref child) = **child {
                    count += visit_quad2(child, depth + 1, max_depth, model);
                }
            }
        }
    }
    count
}

fn visit_quad(node: &Node<bool>, depth: u32, max_depth: u32, model: &mut Model) -> usize {
    let mut count = 0;
    match node.ty {
        NodeType::Leaf(_) => {
            return 0;
        }
        NodeType::Internal => {
            let mut true_count = 0;
            for i in 0..8 {
                if let Some(ref child) = *node.children[i] {
                    if let NodeType::Leaf(value) = child.ty {
                        if value {
                            true_count += 1;
                        }
                    }
                }
            }
            if true_count == 8 {
                panic!("all true");
            }
            /*
            if true_count != 0 {
                count += 1;

                let b = node.bounds[0];
                let idx = from_voxel_idx([b.x, b.y, b.z]);
                model.add_cube(idx);
            }
            */

            for child in node.children.iter() {
                if let Some(ref child) = **child {
                    count += visit_quad(child, depth + 1, max_depth, model);
                }
            }
        }
    }
    count
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
            if let Some(value) = self.inner.get(coord1) {
                *value
            } else {
                false
            }
        } else {
            false
        }
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
        if let Some(coord1) = to_voxel_idx(coord) {
            self.bb.add(coord);
            if self.occupied(coord) {
                return false;
            }
            self.inner.insert(coord1, true).is_ok()
        } else {
            false
        }
    }

    fn to_model(&self) -> Model {
        let mut model = Model::default();
        visit_quad(
            &self.inner.root(),
            0,
            self.inner.max_lod_level(),
            &mut model,
        );

        let quad_count = visit_quad2(
            &self.inner.root(),
            0,
            self.inner.max_lod_level(),
            &mut model,
        );
        info!("quad_count: {}", quad_count);
        model
    }
}

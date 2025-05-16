use log::*;
use std::collections::BTreeSet;
use std::num::NonZeroU32;
use std::rc::Rc;

use super::{BoundingBox, Model, Voxel, VoxelIdx};
use binary_greedy_meshing as bgm;
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

fn fill_quad3(base: &Node<bool>, node: &Node<bool>, voxels: &mut [u16; bgm::CS_P3]) -> usize {
    let mut count = 0;

    match node.ty {
        NodeType::Leaf(value) => {
            let b = node.bounds;
            for x in b[0].x..b[1].x {
                let x1 = (x - base.bounds[0].x) as usize;
                for y in b[0].y..b[1].y {
                    let y1 = (y - base.bounds[0].y) as usize;
                    for z in b[0].z..b[1].z {
                        let z1 = (z - base.bounds[0].z) as usize;

                        let idx = bgm::pad_linearize(x1, y1, z1);
                        if value {
                            voxels[idx] = 1;
                        }
                        count += 1;
                    }
                }
            }
        }
        NodeType::Internal => {
            for child in node.children.iter() {
                if let Some(ref child) = **child {
                    count += fill_quad3(base, child, voxels);
                }
            }
        }
    }

    count
}

fn visit_quad_bgm(node: &Node<bool>, model: &mut Model) -> usize {
    if let NodeType::Leaf(_) = node.ty {
        return 0;
    }

    let b = node.bounds;
    let size = b[1].x - b[0].x;
    if size > 32 {
        let mut count = 0;
        for child in node.children.iter() {
            if let Some(ref child) = **child {
                count += visit_quad_bgm(child, model);
            }
        }
        return count;
    }

    let mut voxels = [0; bgm::CS_P3];

    fill_quad3(node, node, &mut voxels);
    let mut mesh_data = bgm::MeshData::new();
    bgm::mesh(&voxels, &mut mesh_data, BTreeSet::default());

    let decode_quad = |quad: u64| -> (VoxelIdx, [i32; 2]) {
        let x = (quad & 0b111111) as u32;
        let y = ((quad >> 6) & 0b111111) as u32;
        let z = ((quad >> 12) & 0b111111) as u32;
        let w = ((quad >> 18) & 0b111111) as u32;
        let h = ((quad >> 24) & 0b111111) as u32;

        let ox = b[0].x + x as u32;
        let oy = b[0].y + y as u32;
        let oz = b[0].z + z as u32;

        let idx = from_voxel_idx([ox, oy, oz]);
        (idx, [w as i32, h as i32])
    };

    // Up, Down, Right, Left, Front, Back, in this order. (assuming right handed Y up)

    for quad in mesh_data.quads[0].iter() {
        // Up
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([w, 0, h]));
    }
    for quad in mesh_data.quads[1].iter() {
        // Down
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([-w, 0, h]));
    }
    for quad in mesh_data.quads[2].iter() {
        // Right
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([0, -w, h]));
    }
    for quad in mesh_data.quads[3].iter() {
        // Left
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([0, w, h]));
    }
    for quad in mesh_data.quads[4].iter() {
        // Front
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([-w, h, 0]));
    }
    for quad in mesh_data.quads[5].iter() {
        // Back
        let (idx, [w, h]) = decode_quad(*quad);
        model.add_face(idx, VoxelIdx::from([w, h, 0]));
    }

    let mut count = 0;
    for quads in mesh_data.quads.iter() {
        count += quads.len();
    }

    count
}

fn visit_quad_naive(node: &Node<bool>, depth: u32, max_depth: u32, model: &mut Model) -> usize {
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
                    count += visit_quad_naive(child, depth + 1, max_depth, model);
                }
            }
        }
    }
    count
}

#[allow(unused)]
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

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut model = Model::default();

        let quad_count = if false {
            visit_quad_naive(
                &self.inner.root(),
                0,
                self.inner.max_lod_level(),
                &mut model,
            )
        } else {
            visit_quad_bgm(&self.inner.root(), &mut model)
        };

        info!("quad_count: {}", quad_count);
        vec![Rc::new(model)]
    }
}

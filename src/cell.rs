use super::{Model, VoxelIdx};
use binary_greedy_meshing as bgm;
use std::cell::Cell;
use std::collections::BTreeSet;

pub const CELL_SIZE: usize = 32;

#[derive(Debug)]
pub struct BGMCell {
    // 32 x 32 x 32 cell, 1 bit per voxel
    pub dirty: Cell<bool>,
    data: [u32; CELL_SIZE * CELL_SIZE],
}

impl std::default::Default for BGMCell {
    fn default() -> Self {
        Self {
            dirty: Cell::new(false),
            data: [0; 1024],
        }
    }
}

impl BGMCell {
    fn index(x: usize, y: usize) -> usize {
        (x << 5) | y
    }
    fn bitmask(z: usize) -> u32 {
        1 << z
    }
    pub fn get(&self, x: usize, y: usize, z: usize) -> bool {
        let idx = Self::index(x, y);
        let mask = Self::bitmask(z);
        (self.data[idx] & mask) != 0
    }
    pub fn set(&mut self, x: usize, y: usize, z: usize) {
        let idx = Self::index(x, y);
        let mask = Self::bitmask(z);
        self.data[idx] |= mask;
        self.dirty.set(true);
    }

    fn fill_bgm(&self, voxels: &mut [u16; bgm::CS_P3]) {
        for i in 0..1024 {
            let data = self.data[i];
            let x = (i >> 5) & 0b11111;
            let y = (i & 0b11111) as usize;
            let offset = (x + 1) * bgm::CS_P + (y + 1) * bgm::CS_P2;

            for z in 0..CELL_SIZE {
                let v = if data & (1 << z) > 0 { 1 } else { 0 };
                let idx = offset + z + 1;
                voxels[idx] = v;
            }
        }
    }

    pub fn to_model(
        &self,
        base: VoxelIdx,
        voxels: &mut [u16; bgm::CS_P3],
        model: &mut Model,
    ) -> usize {
        self.fill_bgm(voxels);
        let mut mesh_data = bgm::MeshData::new();
        bgm::mesh(voxels, &mut mesh_data, BTreeSet::default());

        let decode_quad = |quad: u64| -> (VoxelIdx, [i32; 2]) {
            let x = (quad & 0b111111) as i32;
            let y = ((quad >> 6) & 0b111111) as i32;
            let z = ((quad >> 12) & 0b111111) as i32;
            let w = ((quad >> 18) & 0b111111) as i32;
            let h = ((quad >> 24) & 0b111111) as i32;

            let ox = base.idx[0] + x;
            let oy = base.idx[1] + y;
            let oz = base.idx[2] + z;

            (VoxelIdx::from([ox, oy, oz]), [w as i32, h as i32])
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
}

const BITS: u64 = 21;
const MASK: u64 = (1 << BITS) - 1;
pub fn chunk_idx(coord: VoxelIdx) -> u64 {
    let x = coord.idx[0].div_euclid(CELL_SIZE as i32);
    let y = coord.idx[1].div_euclid(CELL_SIZE as i32);
    let z = coord.idx[2].div_euclid(CELL_SIZE as i32);

    let pack_axis = |v: i32| -> u64 { unsafe { std::mem::transmute::<_, u32>(v) as u64 } };

    (pack_axis(x) << (BITS * 2)) | (pack_axis(y) << BITS) | pack_axis(z)
}

pub fn chunk_base(key: u64) -> VoxelIdx {
    let xpart = key >> (BITS * 2);
    let ypart = (key >> BITS) & MASK;
    let zpart = key & MASK;

    let unpack_axis = |v: u64| -> i32 { unsafe { std::mem::transmute(v as u32) } };

    let x = unpack_axis(xpart) << 5;
    let y = unpack_axis(ypart) << 5;
    let z = unpack_axis(zpart) << 5;
    VoxelIdx::from([x, y, z])
}

fn rem_euclid(a: i32, b: i32) -> i32 {
    let r = a % b;
    if r < 0 {
        r + b
    } else {
        r
    }
}

pub fn cell_idx(coord: VoxelIdx) -> [usize; 3] {
    let x = rem_euclid(coord.idx[0], CELL_SIZE as i32) as usize;
    let y = rem_euclid(coord.idx[1], CELL_SIZE as i32) as usize;
    let z = rem_euclid(coord.idx[2], CELL_SIZE as i32) as usize;
    [x, y, z]
}

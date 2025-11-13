use super::{Model, VoxelIdx};
use bgm::Quad;
use binary_greedy_meshing as bgm;
use std::collections::*;

pub const CELL_SIZE_BITS: i32 = 5;
pub const CELL_SIZE: usize = 32;

pub type Mesher = bgm::Mesher<CELL_SIZE>;
pub const CS_P: usize = CELL_SIZE + 2;
pub const CS_P2: usize = CS_P * CS_P;
pub const CS_P3: usize = CS_P2 * CS_P;

#[derive(Debug, Clone)]
pub struct BGMCell {
    // 32 x 32 x 32 cell, 1 bit per voxel
    data: [u32; CELL_SIZE * CELL_SIZE],
}

impl std::default::Default for BGMCell {
    fn default() -> Self {
        Self { data: [0; 1024] }
    }
}

pub fn decode_quad(quad: Quad) -> (VoxelIdx, [i32; 2]) {
    let quad = quad.0;
    let x = (quad & 0b111111) as i32;
    let y = ((quad >> 6) & 0b111111) as i32;
    let z = ((quad >> 12) & 0b111111) as i32;
    let w = ((quad >> 18) & 0b111111) as i32;
    let h = ((quad >> 24) & 0b111111) as i32;
    (VoxelIdx::from([x, y, z]), [w as i32, h as i32])
}

pub fn cell_neighbors(coord: VoxelIdx) -> Vec<VoxelIdx> {
    let mut v = Vec::with_capacity(6);
    if coord[0] > 0 {
        v.push(coord + VoxelIdx::new([-1, 0, 0]));
    }
    if coord[0] < CELL_SIZE as i32 - 1 {
        v.push(coord + VoxelIdx::new([1, 0, 0]));
    }
    if coord[1] > 0 {
        v.push(coord + VoxelIdx::new([0, -1, 0]));
    }
    if coord[1] < CELL_SIZE as i32 - 1 {
        v.push(coord + VoxelIdx::new([0, 1, 0]));
    }
    if coord[2] > 0 {
        v.push(coord + VoxelIdx::new([0, 0, -1]));
    }
    if coord[2] < CELL_SIZE as i32 - 1 {
        v.push(coord + VoxelIdx::new([0, 0, 1]));
    }
    v
}

/// Call `op(index)` exactly once for every boundary voxel.
#[inline(always)]
fn for_each_shell<F: FnMut(usize, usize, usize)>(n: usize, mut op: F) {
    let max = n - 1;

    for &z in &[0, max] {
        for y in 0..n {
            for x in 0..n {
                op(x, y, z);
            }
        }
    }

    for z in 1..max {
        for &y in &[0, max] {
            for x in 0..n {
                op(x, y, z);
            }
        }

        for y in 1..max {
            op(0, y, z);
            op(max, y, z);
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
    }
    pub fn clear(&mut self, x: usize, y: usize, z: usize) {
        let idx = Self::index(x, y);
        let mask = Self::bitmask(z);
        self.data[idx] &= !mask;
    }

    pub fn fill_bgm_solid(&self, voxels: &mut [u16; CS_P3]) {
        use std::collections::VecDeque;
        // first bit: cells, seconds bit: to_visit

        const VALUE_MASK: u16 = 0b01;
        const TOVISIT_MASK: u16 = 0b10;
        self.fill_bgm(voxels, TOVISIT_MASK);

        for_each_shell(34, |x, y, z| {
            let idx = z + x * CS_P + y * CS_P2;
            voxels[idx] = TOVISIT_MASK;
        });

        // bfs surface search
        let mut queue = VecDeque::new();
        queue.push_back(0);

        const OFFSETS: [usize; 3] = [1, CS_P, CS_P2];

        while let Some(idx) = queue.pop_front() {
            if voxels[idx] & TOVISIT_MASK == 0 {
                continue;
            }
            voxels[idx] &= !TOVISIT_MASK;

            let mut visit = |idx: usize| {
                let v = voxels[idx];
                // filled, skip
                if v & VALUE_MASK != 0 {
                    return;
                }
                // already visited, skip
                if v & TOVISIT_MASK == 0 {
                    return;
                }
                queue.push_back(idx);
            };

            for offset in OFFSETS {
                if idx + offset < CS_P3 {
                    visit(idx + offset);
                }
                if idx >= offset {
                    visit(idx - offset);
                }
            }
        }

        // TOVISIT to value
        for i in 0..CS_P3 {
            if voxels[i] & TOVISIT_MASK != 0 {
                voxels[i] = VALUE_MASK;
            }
        }
    }

    pub fn fill_bgm(&self, voxels: &mut [u16; CS_P3], xor: u16) {
        for i in 0..1024 {
            let data = self.data[i];
            let x = (i >> 5) & 0b11111;
            let y = (i & 0b11111) as usize;
            let offset = 1 + (x + 1) * CS_P + (y + 1) * CS_P2;

            for z in 0..CELL_SIZE {
                let idx = offset + z;
                voxels[idx] = ((data >> z) & 1) as u16 ^ xor;
            }
        }
    }

    pub fn to_model(&self, voxels: &mut [u16; CS_P3], model: &mut Model) -> usize {
        // self.fill_bgm(voxels, 0);
        self.fill_bgm_solid(voxels);
        let mut mesher = Mesher::new();
        let transparent = BTreeSet::default();
        mesher.mesh(voxels, &transparent);

        // Up, Down, Right, Left, Front, Back, in this order. (assuming right handed Y up)

        for quad in mesher.quads[0].iter() {
            // Up
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([w, 0, h]));
        }
        for quad in mesher.quads[1].iter() {
            // Down
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([-w, 0, h]));
        }
        for quad in mesher.quads[2].iter() {
            // Right
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([0, -w, h]));
        }
        for quad in mesher.quads[3].iter() {
            // Left
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([0, w, h]));
        }
        for quad in mesher.quads[4].iter() {
            // Front
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([-w, h, 0]));
        }
        for quad in mesher.quads[5].iter() {
            // Back
            let (idx, [w, h]) = decode_quad(*quad);
            model.add_face(idx, VoxelIdx::from([w, h, 0]));
        }

        let mut count = 0;
        for quads in mesher.quads.iter() {
            count += quads.len();
        }

        count
    }

    pub fn neighbors_sum(&self, coord: VoxelIdx) -> u8 {
        cell_neighbors(coord)
            .iter()
            .map(|n| {
                let [x, y, z] = cell_idx(*n);
                if self.get(x, y, z) {
                    1
                } else {
                    0
                }
            })
            .sum()
    }

    pub fn simplify(&mut self) -> usize {
        let mut v = VecDeque::new();

        for x in 0..CELL_SIZE {
            for y in 0..CELL_SIZE {
                for z in 0..CELL_SIZE {
                    let val = self.get(x, y, z);
                    let sum = self.neighbors_sum(VoxelIdx::new([x as i32, y as i32, z as i32]));
                    if val || sum > 0 {
                        v.push_back(VoxelIdx::new([x as i32, y as i32, z as i32]));
                    }
                }
            }
        }

        let mut mutations = 0;
        while let Some(coord) = v.pop_front() {
            let [x, y, z] = cell_idx(coord);

            let val = self.get(x, y, z);
            let sum = self.neighbors_sum(coord);
            if !val && sum > 3 {
                self.set(x, y, z);
                mutations += 1;
            } else if val && sum < 3 {
                self.clear(x, y, z);
                mutations += 1;
            } else {
                continue;
            }

            for n in cell_neighbors(coord) {
                v.push_back(n);
            }
            v.push_back(coord);
        }

        mutations
    }
}

const BITS: u64 = 16;
const MASK: u64 = (1 << BITS) - 1;
pub fn chunk_idx(coord: VoxelIdx) -> u64 {
    let x = coord.idx[0].div_euclid(CELL_SIZE as i32);
    let y = coord.idx[1].div_euclid(CELL_SIZE as i32);
    let z = coord.idx[2].div_euclid(CELL_SIZE as i32);

    // i16로 좁혀 부호 유지 → u16로 재해석 → u64
    let pack_axis = |v: i32| -> u64 { ((v as i16) as u16) as u64 };

    ((pack_axis(x) & MASK) << (BITS * 2))
        | ((pack_axis(y) & MASK) << BITS)
        |  (pack_axis(z) & MASK)
}

pub fn chunk_base(key: u64) -> VoxelIdx {
    let xpart = (key >> (BITS * 2)) & MASK; // ★ 마스킹 추가
    let ypart = (key >> BITS) & MASK;
    let zpart = key & MASK;

    let unpack_axis = |v: u64| -> i32 { (v as u16 as i16) as i32 };

    let x = unpack_axis(xpart) * (CELL_SIZE as i32); // ★ CELL_SIZE 사용
    let y = unpack_axis(ypart) * (CELL_SIZE as i32);
    let z = unpack_axis(zpart) * (CELL_SIZE as i32);
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

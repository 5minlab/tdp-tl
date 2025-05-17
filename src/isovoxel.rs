use super::{BoundingBox, Model, Voxel, VoxelIdx};

use super::cell::*;
use ahash::AHashMap;
use std::rc::Rc;

#[derive(Default)]
pub struct IsoVoxel {
    chunks: AHashMap<u64, BGMCell>,
    bb: BoundingBox,
}

struct ChunkView<'a> {
    cell: &'a BGMCell,
}

impl<'a> isosurface::source::Source for ChunkView<'a> {
    fn sample(&self, x: f32, y: f32, z: f32) -> f32 {
        let x = (x * 31.0) as usize;
        let y = (y * 31.0) as usize;
        let z = (z * 31.0) as usize;
        if self.cell.get(x, y, z) {
            1.0
        } else {
            0.0
        }
    }
}

impl Voxel for IsoVoxel {
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

        for (&idx, cell) in self.chunks.iter() {
            let base = chunk_base(idx);
            let view = ChunkView { cell };

            let mut model = Model::default();
            let mut cubes = isosurface::marching_cubes::MarchingCubes::new(32);

            let mut vertices = vec![];
            let mut indices = vec![];
            cubes.extract(&view, &mut vertices, &mut indices);

            let vertex_count = vertices.len() / 3;

            let bx = base.idx[0] as f32;
            let by = base.idx[1] as f32;
            let bz = base.idx[2] as f32;

            for i in 0..vertex_count {
                let x = vertices[i * 3] * 32.0 + bx;
                let y = vertices[i * 3 + 1] * 32.0 + by;
                let z = vertices[i * 3 + 2] * 32.0 + bz;
                model.raw_vertices.push([x, y, z]);
            }

            let triangle_count = indices.len() / 3;
            for i in 0..triangle_count {
                let i1 = indices[i * 3];
                let i2 = indices[i * 3 + 1];
                let i3 = indices[i * 3 + 2];

                assert!(i1 < vertex_count as u32);
                assert!(i2 < vertex_count as u32);
                assert!(i3 < vertex_count as u32);

                model.raw_triangles.push([i1, i2, i3]);
            }

            models.push(Rc::new(model));
        }

        models
    }
}

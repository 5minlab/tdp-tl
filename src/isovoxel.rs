use super::{BoundingBox, Model, Voxel, VoxelIdx};

use super::cell::*;
use ahash::AHashMap;
use isosurface::distance::*;
use std::rc::Rc;

#[derive(Default)]
pub struct IsoVoxel {
    chunks: AHashMap<u64, BGMCell>,
    bb: BoundingBox,
}

struct ChunkView<'a> {
    cell: &'a BGMCell,
}

impl<'a> isosurface::sampler::Sample<Signed> for ChunkView<'a> {
    fn sample(&self, p: isosurface::math::vector::Vec3) -> Signed {
        // 0.0 to 0, 1.0 to 31
        let x = ((p.x - std::f32::EPSILON) * CELL_SIZE as f32) as usize;
        let y = ((p.y - std::f32::EPSILON) * CELL_SIZE as f32) as usize;
        let z = ((p.z - std::f32::EPSILON) * CELL_SIZE as f32) as usize;
        if self.cell.get(x, y, z) {
            Signed(1.0)
        } else {
            Signed(-1.0)
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
        const CELL_SIZE_F32: f32 = CELL_SIZE as f32;

        for (&idx, cell) in self.chunks.iter() {
            let base = chunk_base(idx);
            let view = ChunkView { cell };

            let mut model = Model::default();
            let mut cubes = isosurface::MarchingCubes::new(CELL_SIZE / 2);

            let mut vertices = vec![];
            let mut indices = vec![];
            cubes.extract(
                &view,
                &mut isosurface::extractor::IndexedVertices::new(&mut vertices, &mut indices),
            );

            let vertex_count = vertices.len() / 3;

            let bx = base.idx[0] as f32;
            let by = base.idx[1] as f32;
            let bz = base.idx[2] as f32;

            for i in 0..vertex_count {
                let x = vertices[i * 3] * CELL_SIZE_F32 + bx;
                let y = vertices[i * 3 + 1] * CELL_SIZE_F32 + by;
                let z = vertices[i * 3 + 2] * CELL_SIZE_F32 + bz;
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

use super::{BoundingBox, Model, Voxel, VoxelIdx};

use super::cell::*;
use super::chunkedvoxel::ChunkedBase;
use isosurface::distance::*;
use std::rc::Rc;

#[derive(Default)]
pub struct IsoVoxel {
    base: ChunkedBase,
}

struct ChunkView<'a> {
    parent: &'a IsoVoxel,
    base: VoxelIdx,
    cell: &'a BGMCell,
}

impl<'a> isosurface::sampler::Sample<Signed> for ChunkView<'a> {
    fn sample(&self, p: isosurface::math::vector::Vec3) -> Signed {
        // 0.0 to 0, 1.0 to 32 + 1
        let x = (p.x * CELL_SIZE as f32 + 0.5) as usize;
        let y = (p.y * CELL_SIZE as f32 + 0.5) as usize;
        let z = (p.z * CELL_SIZE as f32 + 0.5) as usize;

        if x == 0
            || y == 0
            || z == 0
            || x == (CELL_SIZE + 1)
            || y == (CELL_SIZE + 1)
            || z == (CELL_SIZE + 1)
        {
            let bx = self.base[0] as u32;
            let by = self.base[1] as u32;
            let bz = self.base[2] as u32;

            let worldpos = VoxelIdx::new([
                bx as i32 + x as i32 - 1,
                by as i32 + y as i32 - 1,
                bz as i32 + z as i32 - 1,
            ]);
            if self.parent.occupied(worldpos) {
                return Signed(1.0);
            } else {
                return Signed(-1.0);
            }
        }

        if self.cell.get(x - 1, y - 1, z - 1) {
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
        const CELL_SIZE_F32: f32 = CELL_SIZE as f32;

        for (&idx, cell) in self.base.chunks.iter() {
            let base = chunk_base(idx);
            let view = ChunkView {
                parent: self,
                base,
                cell,
            };

            let mut model = Model::default();
            let mut cubes = isosurface::MarchingCubes::new(CELL_SIZE + 2);

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

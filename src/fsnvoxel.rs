use super::{BoundingBox, Model, Voxel, VoxelIdx};

use super::cell::*;
use super::chunkedvoxel::ChunkedBase;
use fast_surface_nets::ndshape::{ConstShape, ConstShape3u32};
use fast_surface_nets::{surface_nets, SurfaceNetsBuffer};
use std::rc::Rc;

// A 16^3 chunk with 1-voxel boundary padding.
const PADDED_SIZE: u32 = CELL_SIZE as u32 + 2;
type ChunkShape = ConstShape3u32<PADDED_SIZE, PADDED_SIZE, PADDED_SIZE>;

#[derive(Default)]
pub struct FSNVoxel {
    base: ChunkedBase,
}

impl Voxel for FSNVoxel {
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

        for (&idx, cell) in self.base.chunks.iter() {
            let base = chunk_base(idx);
            let bx = base[0] as u32;
            let by = base[1] as u32;
            let bz = base[2] as u32;

            let mut sdf = [-1.0; ChunkShape::USIZE];
            for i in 0u32..ChunkShape::SIZE {
                let [x, y, z] = ChunkShape::delinearize(i);
                if x == 0
                    || y == 0
                    || z == 0
                    || x == (PADDED_SIZE - 1)
                    || y == (PADDED_SIZE - 1)
                    || z == (PADDED_SIZE - 1)
                {
                    let worldpos = VoxelIdx::new([
                        bx as i32 + x as i32 - 1,
                        by as i32 + y as i32 - 1,
                        bz as i32 + z as i32 - 1,
                    ]);
                    if self.occupied(worldpos) {
                        sdf[i as usize] = 1.0;
                    }
                    continue;
                }
                if cell.get(x as usize - 1, y as usize - 1, z as usize - 1) {
                    sdf[i as usize] = 1.0;
                }
            }

            let mut mesh = SurfaceNetsBuffer::default();
            surface_nets(&sdf, &ChunkShape {}, [0; 3], [33; 3], &mut mesh);

            let mut model = Model::default();

            let vertex_count = mesh.positions.len();
            for i in 0..vertex_count {
                let pos = mesh.positions[i];

                model.raw_vertices.push([
                    pos[0] + bx as f32,
                    pos[1] + by as f32,
                    pos[2] + bz as f32,
                ]);
                model.raw_normals.push(mesh.normals[i]);
            }

            let triangle_count = mesh.indices.len() / 3;
            for i in 0..triangle_count {
                let i1 = mesh.indices[i * 3];
                let i2 = mesh.indices[i * 3 + 1];
                let i3 = mesh.indices[i * 3 + 2];

                model.raw_triangles.push([i1 as u32, i2 as u32, i3 as u32]);
            }

            models.push(Rc::new(model));
        }

        models
    }
}

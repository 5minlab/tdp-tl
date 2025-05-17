use super::{BoundingBox, Model, Voxel, VoxelIdx};

use super::cell::*;
use ahash::AHashMap;
use std::rc::Rc;

use transvoxel::voxel_source::*;

#[derive(Default)]
pub struct LodVoxel {
    chunks: AHashMap<u64, BGMCell>,
    bb: BoundingBox,
}

impl<'a> DataField<f32, f32> for &'a LodVoxel {
    fn get_data(&mut self, x: f32, y: f32, z: f32) -> f32 {
        let coord = VoxelIdx::new([x as i32, y as i32, z as i32]);
        if self.occupied(coord) {
            1.0
        } else {
            0.0
        }
    }
}

impl Voxel for LodVoxel {
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

        for (&idx, _cell) in self.chunks.iter() {
            use transvoxel::prelude::*;

            let base = chunk_base(idx);
            let mut model = Model::default();

            let threshold = 0.5f32;

            let subdivisions = 16;
            let block = Block::from(
                [base.idx[0] as f32, base.idx[1] as f32, base.idx[2] as f32],
                32.0,
                subdivisions,
            );
            let source = WorldMappingVoxelSource {
                field: &*self,
                block: &block,
            };
            let transition_sides = transition_sides::no_side();

            // Finally, you can run the mesh extraction:
            use transvoxel::generic_mesh::GenericMeshBuilder;
            let builder = GenericMeshBuilder::new();
            let builder = extract(source, &block, threshold, transition_sides, builder);
            let mesh = builder.build();

            assert_eq!(mesh.positions.len(), mesh.normals.len());

            let vertex_count = mesh.positions.len() / 3;
            for i in 0..vertex_count {
                let x = mesh.positions[i * 3];
                let y = mesh.positions[i * 3 + 1];
                let z = mesh.positions[i * 3 + 2];
                model.raw_vertices.push([x, y, z]);

                let mut x = mesh.normals[i * 3];
                let y = mesh.normals[i * 3 + 1];
                let z = mesh.normals[i * 3 + 2];
                // TODO: fix normals
                if x == 0.0 && y == 0.0 && z == 0.0 {
                    x = 1.0;
                }

                model.raw_normals.push([x, y, z]);
            }

            let triangle_count = mesh.triangle_indices.len() / 3;
            for i in 0..triangle_count {
                let i1 = mesh.triangle_indices[i * 3];
                let i2 = mesh.triangle_indices[i * 3 + 1];
                let i3 = mesh.triangle_indices[i * 3 + 2];

                assert!(i1 < vertex_count);
                assert!(i2 < vertex_count);
                assert!(i3 < vertex_count);

                model.raw_triangles.push([i1 as u32, i2 as u32, i3 as u32]);
            }

            models.push(Rc::new(model));
        }

        models
    }
}

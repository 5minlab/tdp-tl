use super::{BoundingBox, Model, Voxel, VoxelIdx, UNIT};

use anyhow::Result;
use nanovdb_sys::*;
use std::rc::Rc;

use transvoxel::voxel_source::*;

// const LOD_LEVEL: i32 = 2;

#[derive(Default)]
pub struct VDBVoxel {
    grid: BuildGrid,
    // grid2: BuildGrid,

    bb: BoundingBox,
}

pub struct MeshConfig<'a> {
    grid: &'a BuildGrid,
    scaleshift: i32,
}

impl<'a> MeshConfig<'a> {
    fn get(&self, x: f32, y: f32, z: f32) -> f32 {
        let x = (x as i32) >> self.scaleshift;
        let y = (y as i32) >> self.scaleshift;
        let z = (z as i32) >> self.scaleshift;
        self.grid.get(x, y, z) as f32
    }
}

impl<'a> DataField<f32, f32> for &'a MeshConfig<'a> {
    fn get_data(&mut self, x: f32, y: f32, z: f32) -> f32 {
        self.get(x, y, z)
    }
}

fn build_mesh<'a>(
    grid: &'a BuildGrid,
    base: VoxelIdx,
    size: usize,
    scaleshift: i32,
) -> transvoxel::generic_mesh::Mesh<f32> {
    /*
    transvoxel::generic_mesh::Mesh {
        positions: vec![],
        normals: vec![],
        triangle_indices: vec![],
    }
    */
    use transvoxel::prelude::*;
    let threshold = (1 << (size * 3)) as f32 / 2.0;
    let subdivisions = size;

    let base = base.shift_up(scaleshift);
    let size = (size << scaleshift) as f32;

    let leaf = MeshConfig {
        grid: &grid,
        scaleshift,
    };
    let block = Block::from(base.f32(), size as f32, subdivisions);
    let source = WorldMappingVoxelSource {
        field: &leaf,
        block: &block,
    };
    let transition_sides = transition_sides::no_side();

    // Finally, you can run the mesh extraction:
    use transvoxel::generic_mesh::GenericMeshBuilder;
    let builder = GenericMeshBuilder::new();
    let builder = extract(source, &block, threshold, transition_sides, builder);
    builder.build()
}

impl Voxel for VDBVoxel {
    fn ranges(&self) -> usize {
        0
    }

    fn bounding_box(&self) -> &BoundingBox {
        &self.bb
        // &self.base.bb
    }

    fn occupied(&self, coord: VoxelIdx) -> bool {
        self.grid.get(coord[0], coord[1], coord[2]) != 0
        // self.base.occupied(coord)
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
        if self.occupied(coord) {
            return false;
        }
        self.grid.set(coord[0], coord[1], coord[2], 1);

        /*
        let coord2 = coord.shift_down(LOD_LEVEL);
        self.grid2.add(coord2[0], coord2[1], coord2[2], 1);
        */
        true
    }

    fn to_model(&mut self) -> Vec<Rc<Model>> {
        let mut models = vec![];

        let grid = &self.grid;
        let scaleshift = 0;

        let count = grid.iter2_init();
        for idx in 0..count {
            let base = VoxelIdx {
                idx: grid.iter2_get0(idx),
            };
            let mesh = build_mesh(grid, base, NODE1_DIM, scaleshift);
            if mesh.positions.len() == 0 {
                continue;
            }

            assert_eq!(mesh.positions.len(), mesh.normals.len());
            // eprintln!("base: {:?}, positions: {}, triangles: {}", base, mesh.positions.len(), mesh.triangle_indices.len());

            let mut model = Model::default();
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

    fn debug(&self, filename: &str) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        let grid = &self.grid;
        let scaleshift = 0;
        let writer = std::fs::File::create(filename)?;
        let mut writer = std::io::BufWriter::new(writer);

        // type
        writer.write_u32::<LittleEndian>(1)?;
        writer.write_f32::<LittleEndian>(UNIT)?;

        let count = grid.iter2_init();
        writer.write_u32::<LittleEndian>(count as u32)?;
        for idx in 0..count {
            let base = VoxelIdx {
                idx: grid.iter2_get0(idx),
            };
            let mesh = build_mesh(grid, base, NODE1_DIM, scaleshift);
            if mesh.positions.len() == 0 {
                continue;
            }

            writer.write_u32::<LittleEndian>(mesh.positions.len() as u32)?;
            for i in 0..(mesh.positions.len() / 3) {
                let x = mesh.positions[i * 3];
                let y = mesh.positions[i * 3 + 1];
                let z = mesh.positions[i * 3 + 2];
                writer.write_f32::<LittleEndian>(x)?;
                writer.write_f32::<LittleEndian>(y)?;
                writer.write_f32::<LittleEndian>(z)?;
            }

            writer.write_u32::<LittleEndian>(mesh.normals.len() as u32)?;
            for i in 0..mesh.normals.len() {
                writer.write_f32::<LittleEndian>(mesh.normals[i])?;
            }

            writer.write_u32::<LittleEndian>(mesh.triangle_indices.len() as u32)?;
            for i in 0..mesh.triangle_indices.len() {
                let idx = mesh.triangle_indices[i] as u32;
                assert!(idx < mesh.positions.len() as u32);
                writer.write_u32::<LittleEndian>(idx)?;
            }
        }

        Ok(())
    }
}

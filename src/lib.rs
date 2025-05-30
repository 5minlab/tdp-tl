use anyhow::Result;
use log::*;
use nalgebra::Point3;
use nalgebra::Vector3;
use simple_stopwatch::Stopwatch;
use std::fs::File;
use std::rc::Rc;
use std::sync::*;

mod cell;
mod voxelidx;
pub use voxelidx::VoxelIdx;
mod rangesetvoxel;
pub use rangesetvoxel::RangeSetVoxel;
mod monotonicvoxel;
pub use monotonicvoxel::MonotonicVoxel;
mod svovoxel;
pub use svovoxel::SVOVoxel;
mod chunkedvoxel;
pub use chunkedvoxel::ChunkedVoxel;
mod lodvoxel;
pub use lodvoxel::LodVoxel;
mod isovoxel;
pub use isovoxel::IsoVoxel;
mod fsnvoxel;
pub use fsnvoxel::FSNVoxel;
#[cfg(feature = "nanovdb")]
mod vdbvoxel;
#[cfg(feature = "nanovdb")]
pub use vdbvoxel::VDBVoxel;

mod extrude;
pub use extrude::*;
mod gcode;
pub use cell::*;
pub use gcode::*;

impl std::ops::Index<usize> for VoxelIdx {
    type Output = i32;

    fn index(&self, index: usize) -> &Self::Output {
        &self.idx[index]
    }
}

#[derive(Default, Debug)]
pub struct BoundingBox {
    bound_min: VoxelIdx,
    bound_max: VoxelIdx,

    count: usize,
}

impl BoundingBox {
    fn add(&mut self, coord: VoxelIdx) {
        // first block
        if self.count == 0 {
            self.bound_min = coord;
            self.bound_max = coord;
        } else {
            self.bound_min = coord.bb_min(&self.bound_min);
            self.bound_max = coord.bb_max(&self.bound_max);
        }
        self.count += 1;
    }
}

// internal use only
const FPS: usize = 30;

const FILAMENT_DIAMETER: f32 = 1.75f32;
const FILAMENT_CROSS_SECION: f32 =
    0.25f32 * std::f32::consts::PI * FILAMENT_DIAMETER * FILAMENT_DIAMETER;

const NOZZLE_SIZE: f32 = 0.4f32;

pub struct Parameters {
    pub unit: f32,
    pub layer_height: f32,
}

impl Default for Parameters {
    fn default() -> Self {
        let layer_height = 0.2f32;
        Self {
            unit: layer_height / 2.0,
            layer_height,
        }
    }
}

impl Parameters {
    fn intpos(&self, v: f32) -> i32 {
        (v / self.unit).round() as i32
    }

    fn to_intpos(&self, pos: Vector3<f32>) -> VoxelIdx {
        [
            self.intpos(pos[0]),
            self.intpos(pos[1]),
            self.intpos(pos[2]),
        ]
        .into()
    }

    fn write<W: std::io::Write>(&self, mut writer: W) -> Result<()> {
        use byteorder::{LittleEndian, WriteBytesExt};

        writer.write_f32::<LittleEndian>(self.unit)?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WriteOptions {
    #[default]
    None,

    Simplify,
}

pub trait Voxel: Default {
    fn ranges(&self) -> usize;
    fn bounding_box(&self) -> &BoundingBox;
    fn occupied(&self, coord: VoxelIdx) -> bool;
    fn add(&mut self, coord: VoxelIdx) -> bool;
    fn to_model(&mut self) -> Vec<Rc<Model>>;

    fn write_binary<W: std::io::Write>(&mut self, _writer: W) -> Result<()> {
        Ok(())
    }

    fn debug1(&mut self) -> usize {
        1
    }

    fn set_options(&mut self, _options: WriteOptions) {}
}

pub trait StreamingVoxel: Voxel {
    fn write_dirty<W: std::io::Write>(&mut self, writer: W) -> Result<()>;
}

#[derive(Default)]
pub struct Model {
    id: u64,
    offset: VoxelIdx,

    vertices: indexmap::IndexSet<VoxelIdx>,
    faces: Vec<[usize; 4]>,

    raw_vertices: Vec<[f32; 3]>,
    raw_triangles: Vec<[u32; 3]>,
    raw_normals: Vec<[f32; 3]>,
}

impl Model {
    fn add_vert(&mut self, coord: VoxelIdx) -> usize {
        let (idx, _) = self.vertices.insert_full(coord);
        idx
    }

    fn add_face(&mut self, coord: VoxelIdx, dir: VoxelIdx) {
        let (i0, i1, i2, i3) = if dir[0] == 0 {
            let i0 = self.add_vert(coord);
            let i1 = self.add_vert(coord + dir.y());
            let i2 = self.add_vert(coord + dir.yz());
            let i3 = self.add_vert(coord + dir.z());
            (i0, i1, i2, i3)
        } else if dir[1] == 0 {
            let i0 = self.add_vert(coord);
            let i1 = self.add_vert(coord + dir.x());
            let i2 = self.add_vert(coord + dir.xz());
            let i3 = self.add_vert(coord + dir.z());
            (i0, i1, i2, i3)
        } else {
            let i0 = self.add_vert(coord);
            let i1 = self.add_vert(coord + dir.x());
            let i2 = self.add_vert(coord + dir.xy());
            let i3 = self.add_vert(coord + dir.y());
            (i0, i1, i2, i3)
        };

        self.faces.push([i0, i1, i2, i3]);
    }

    pub fn add_cube(&mut self, coord: VoxelIdx) {
        self.add_cube_size(coord, 1);
    }

    fn add_cube_size(&mut self, coord: VoxelIdx, size: i32) {
        // self.add_face(coord, [size, size, 0].into());
        self.add_face(coord, [size, 0, size].into());
        self.add_face(coord, [0, size, size].into());

        let coord = coord + VoxelIdx::unitsize(size);

        self.add_face(coord, [-size, -size, 0].into());
        self.add_face(coord, [-size, 0, -size].into());
        self.add_face(coord, [0, -size, -size].into());
    }

    #[allow(unused)]
    fn serialize_raw(&self, path: &str) -> Result<()> {
        use std::io::Write;

        let w = File::create(path)?;
        let mut w = std::io::BufWriter::new(w);

        for idx in &self.vertices {
            let x = idx[0];
            let y = idx[1];
            let z = idx[2];
            write!(&mut w, "v {} {} {}\n", x, y, z)?;
        }
        for [i0, i1, i2, i3] in &self.faces {
            write!(&mut w, "f {} {} {} {}\n", i0 + 1, i1 + 1, i2 + 1, i3 + 1)?;
        }

        Ok(())
    }
}

pub fn model_serialize_gltf(
    models: &[Rc<Model>],
    path: &str,
    offset: [f32; 3],
    scale: f32,
) -> Result<()> {
    use mesh_tools::{GltfBuilder, Triangle};
    let mut builder = GltfBuilder::new();

    let material = builder.create_basic_material(Some("red".to_owned()), [1.0, 0.2, 0.2, 1.0]);
    let mut nodes = vec![];

    for (i, model) in models.iter().enumerate() {
        if !model.vertices.is_empty() {
            let positions = model
                .vertices
                .iter()
                .map(|idx| Point3::new(idx[0] as f32, idx[2] as f32, -idx[1] as f32))
                .collect::<Vec<_>>();
            let indices = model
                .faces
                .iter()
                .flat_map(|[i0, i1, i2, i3]| {
                    let i0 = *i0 as u32;
                    let i1 = *i1 as u32;
                    let i2 = *i2 as u32;
                    let i3 = *i3 as u32;
                    [Triangle::new(i0, i2, i1), Triangle::new(i0, i3, i2)]
                })
                .collect::<Vec<_>>();

            let name = format!("quads_{}", i);
            let mesh = builder.create_custom_mesh(
                Some(name.clone()),
                &positions,
                &indices,
                None,
                None,
                Some(material),
            );

            let offset = model.offset;
            let offset = [offset[0] as f32, offset[2] as f32, -offset[1] as f32];
            let node = builder.add_node(Some(name.clone()), Some(mesh), Some(offset), None, None);
            nodes.push(node);
        }

        if !model.raw_vertices.is_empty() {
            let positions = model
                .raw_vertices
                .iter()
                .map(|idx| Point3::new(idx[0], idx[2], -idx[1]))
                .collect::<Vec<_>>();

            let indices = model
                .raw_triangles
                .iter()
                .flat_map(|[i0, i1, i2]| {
                    let i0 = *i0;
                    let i1 = *i1;
                    let i2 = *i2;
                    [Triangle::new(i0, i1, i2)]
                })
                .collect::<Vec<_>>();

            let normals = if model.raw_normals.is_empty() {
                None
            } else {
                assert_eq!(positions.len(), model.raw_normals.len());
                Some(
                    model
                        .raw_normals
                        .iter()
                        .map(|idx| nalgebra::Vector3::new(idx[0], idx[2], -idx[1]))
                        .collect::<Vec<_>>(),
                )
            };

            let name = format!("raw_{}", i);
            let mesh = builder.create_custom_mesh(
                Some(name.clone()),
                &positions,
                &indices,
                normals,
                None,
                Some(material),
            );
            let node = builder.add_node(Some(name.clone()), Some(mesh), None, None, None);
            nodes.push(node);
        }
    }

    let root = builder.add_node_with_children(
        Some("root".to_owned()),
        None,
        Some([offset[0], offset[2], -offset[1]]),
        None,
        Some([scale, scale, scale]),
        nodes,
    );

    builder.add_scene(Some("scene".to_owned()), Some(vec![root]));
    builder.export_glb(path)?;

    Ok(())
}

pub fn model_serialize(
    models: &[Rc<Model>],
    path: &str,
    offset: [f32; 3],
    scale: f32,
) -> Result<()> {
    use std::io::Write;

    let w = File::create(path)?;
    let mut w = std::io::BufWriter::new(w);

    let mut o = 1;
    for model in models {
        for idx in &model.vertices {
            let x = idx[0];
            let y = idx[1];
            let z = idx[2];
            write!(
                &mut w,
                "v {:.2} {:.2} {:.2}\n",
                x as f32 * scale + offset[0],
                y as f32 * scale + offset[1],
                z as f32 * scale + offset[2]
            )?;
        }
        for [i0, i1, i2, i3] in &model.faces {
            write!(&mut w, "f {} {} {} {}\n", i0 + o, i1 + o, i2 + o, i3 + o)?;
        }
        o += model.vertices.len();
    }

    Ok(())
}

struct ExtrudeState<V: Voxel + Default> {
    mv: V,
    params: Parameters,

    home: Vector3<f32>,
    pos: Vector3<f32>,
    f: f32,
    e: f32,

    frames: usize,
    dirtycount: usize,

    wall_seconds: f32,
    last_sw: Stopwatch,
}

impl<V: Voxel + Default> std::default::Default for ExtrudeState<V> {
    fn default() -> Self {
        let last_sw = Stopwatch::start_new();
        Self {
            mv: V::default(),
            params: Parameters::default(),

            home: Vector3::new(0.0, 0.0, 0.0),
            pos: Vector3::new(0.0, 0.0, 0.0),
            f: 0.0,
            e: 0.0,

            frames: 0,
            dirtycount: 0,

            wall_seconds: 0.0,
            last_sw,
        }
    }
}

impl<V: Voxel + Default> ExtrudeState<V> {
    fn export(&mut self, out_filename: &str, postfix: &str) -> Result<()> {
        let last_dt = self.last_sw.ms();

        let sw = Stopwatch::start_new();
        let model = self.mv.to_model();
        info!(
            "to_model: took={:.2}ms/{:.2}ms, wall: {:.0}s",
            last_dt,
            sw.ms(),
            self.wall_seconds
        );

        let sw = Stopwatch::start_new();

        let out_filename = if true {
            let filename = format!("{}/gcode_{}.glb", out_filename, postfix);
            model_serialize_gltf(&model, &filename, [-90f32, -90f32, 0f32], self.params.unit)?;

            {
                let filename1 = format!("{}/gcode_{}.bin", out_filename, postfix);
                let file = File::create(&filename1)?;
                let mut writer = std::io::BufWriter::new(file);
                self.params.write(&mut writer)?;
                self.mv.write_binary(&mut writer)?;
            }
            filename
        } else {
            let filename = format!("{}/gcode_{}.obj", out_filename, postfix);
            // let model = mv.to_model();
            // model_serialize(&model, &filename, [-90f32, -90f32, 0f32], UNIT)?;
            filename
        };
        // model.serialize_raw(&out_filename)?;
        info!(
            "Model::serialize: took={:.2}ms, filename={}",
            sw.ms(),
            out_filename
        );

        self.last_sw = Stopwatch::start_new();
        Ok(())
    }

    fn handle_gcode(&mut self, code: GCode1Coord) -> usize {
        if code.major == 92 {
            self.e = code.e.unwrap_or(self.e);
            return 0;
        }

        // unit: millimeters
        // TODO: extract from gcode
        let z_offset: i32 = (self.params.layer_height / self.params.unit) as i32;
        let z_offset_up: i32 = 1;

        // tunables
        let inject_offset_z: f32 = 0.0; // LAYER_HEIGHT / 2.0;

        let mut dst = self.pos;
        let mut dst_e = self.e;
        let mut target_f = self.f;
        if code.major == 0 || code.major == 1 {
            if let Some(x) = code.x {
                dst[0] = x;
            }
            if let Some(y) = code.y {
                dst[1] = y;
            }
            if let Some(z) = code.z {
                dst[2] = z;
            }
            if let Some(e) = code.e {
                dst_e = e;
            }
            if let Some(f) = code.f {
                target_f = f;
            }
        }

        self.f = target_f;
        if dst_e <= self.e {
            self.pos = dst;
            return 0;
        }

        let diff = dst - self.pos;
        // in millimeters
        let len = diff.magnitude();
        if len < std::f32::EPSILON {
            return 0;
        }
        let dir = diff.normalize();

        let seconds = len / (self.f / 60.0);
        self.wall_seconds += seconds;

        // in centimeters
        let delta_e = dst_e - self.e;
        if delta_e <= std::f32::EPSILON {
            return 0;
        }

        // no inter-layer extrusion
        assert!(dir.z <= 0.0, "delta_e={}, dir={:?}", delta_e, dir);

        let z = self.params.intpos(dst[2]);
        let zrange = (z - z_offset)..(z + z_offset_up);

        let oz = self.home + Vector3::new(0.0, 0.0, -inject_offset_z);
        let offsets = [
            oz + Vector3::new(0.0, 0.0, 0.0),
            oz + Vector3::new(dir.y, -dir.x, 0.0) * NOZZLE_SIZE / 8.0,
            oz + Vector3::new(-dir.y, dir.x, 0.0) * NOZZLE_SIZE / 8.0,
            oz + Vector3::new(dir.y, -dir.x, 0.0) * NOZZLE_SIZE / 6.0,
            oz + Vector3::new(-dir.y, dir.x, 0.0) * NOZZLE_SIZE / 6.0,
        ];

        let gen_cells = |from: Vector3<f32>, to: Vector3<f32>| {
            let mut cells = vec![];
            for offset in &offsets {
                let pos = self.params.to_intpos(from + offset);
                let next_pos = self.params.to_intpos(to + offset);
                line_cells(pos, next_pos, &mut cells);
            }
            cells
        };

        // flow rate calculation
        // with 1.75mm filament, calculate volume, in millimeters
        let filament_volume = delta_e * FILAMENT_CROSS_SECION;

        // block volume in cubic millimeters
        let block_volume = self.params.unit.powi(3);

        // TODO: accurate volume calculation
        let total_blocks = filament_volume / block_volume;
        let mut blocks = total_blocks as usize;

        debug!(
            "{:?} -> {:?}, len={}, e={:?}, blocks={}",
            self.pos,
            dst,
            len,
            dst_e - self.e,
            total_blocks
        );

        let cursor = self.pos;

        let max_dist = (NOZZLE_SIZE * 2.0 / self.params.unit) as usize;

        // 1800mm/min, 30mm/s, 0.5mm/frame
        /*
        let step_size = self.f / FPS as f32 / 60.0;

        let blocks_per_step = (total_blocks * step_size / len) as usize;
        while (cursor - dst).magnitude() > step_size {
            let next = cursor + dir * step_size;

            let cells = gen_cells(cursor, next);
            let extrudeed = extrude_at(&mut self.mv, zrange.clone(), &cells, blocks_per_step);
            if extrudeed != blocks_per_step {
                debug!(
                    "extrudeed != blocks_per_step, skipping: {} != {}",
                    extrudeed, blocks_per_step
                );
            }
            cursor = next;
            blocks -= blocks_per_step;

            self.frames += 1;
            self.dirtycount += self.mv.debug1();
        }
        */

        // last segment
        if blocks > 0 {
            let cells = gen_cells(cursor, dst);
            let extrudeed = extrude_at(&mut self.mv, zrange.clone(), max_dist, &cells, blocks);
            if extrudeed != blocks {
                debug!("extrudeed != blocks, skipping: {} != {}", extrudeed, blocks);
            }
            blocks -= extrudeed;
        }

        self.pos = dst;
        self.e = dst_e;
        blocks
    }
}

pub fn generate_gcode<V: Voxel + Default>(
    filename: &str,
    out_filename: &str,
    layer: usize,
    out_layers: bool,
) -> Result<()> {
    let mut state = ExtrudeState::<V>::default();

    let sw = Stopwatch::start_new();
    let parsed = parse_gcode(filename)?;

    if true {
        let mut runner = ExtrudeRunner::<V>::new(parsed);
        info!("meta: {:?}", runner.meta);
        while !runner.step(1.0 / FPS as f32) {
            // runner.state.mv.debug1();
        }
        state = runner.state;
    } else {
        for (_line, item) in parsed {
            match item {
                GCode1::Layer(layer_idx) => {
                    if layer_idx == 0 {
                        continue;
                    }

                    if layer_idx == layer {
                        break;
                    }

                    if out_layers && layer_idx % 10 == 0 {
                        let postfix = format!("{:03}", layer_idx);
                        state.export(out_filename, &postfix)?;
                    }
                }
                GCode1::Coord(coord) => {
                    state.handle_gcode(coord);
                }
                _ => {
                    //
                }
            }
        }
    }
    state.export(out_filename, "full")?;

    let blocks = state.mv.bounding_box().count;
    info!(
        "voxel construction: took={:.2}ms, blocks={}/{}, bps={}, frames={}, {:.1} dirty / frame",
        sw.ms(),
        blocks,
        state.mv.ranges(),
        blocks * 1000 / sw.ms() as usize,
        state.frames,
        state.dirtycount as f32 / state.frames as f32,
    );

    Ok(())
}

struct ExtrudeRunner<V: Voxel> {
    pub meta: GCodeMeta,
    pub state: ExtrudeState<V>,

    pendings: Vec<(usize, GCode1)>,
}

impl<V: Voxel + Default> ExtrudeRunner<V> {
    fn new(mut pendings: Vec<(usize, GCode1)>) -> Self {
        let meta = {
            let comments = pendings
                .iter()
                .filter_map(|(_, code)| {
                    if let GCode1::TypedComment(prefix, value) = code {
                        Some((prefix.as_str(), value.as_str()))
                    } else {
                        None
                    }
                })
                .collect::<Vec<_>>();
            GCodeMeta::from_comments(&comments)
        };
        let mut state = ExtrudeState::<V>::default();

        if let Some((min, max)) = meta.bounding_box {
            // check if coordinate space is center-zero

            let center = (min + max) / 2.0;
            let extents = (max - min) / 2.0;
            if center.x < extents.x / 2.0 {
                state.home = Vector3::new(110.0, 110.0, 0.0);
            }
        }
        pendings.reverse();

        Self {
            meta,
            state,
            pendings,
        }
    }

    fn pos(&self) -> Vector3<f32> {
        self.state.pos + self.state.home
    }

    fn step(&mut self, mut dt: f32) -> bool {
        while dt > std::f32::EPSILON {
            match self.step0(dt) {
                (true, _) => {
                    // no more steps
                    return true;
                }
                (false, step_dt) => {
                    dt -= step_dt;
                }
            }
        }
        false
    }

    // G0, G1, G2, G3
    fn g_0_1(&mut self, cur: GCode1Coord, dt: f32) -> (Option<GCode1Coord>, f32) {
        let prev = GCode1Coord {
            major: cur.major,
            x: Some(self.state.pos[0]),
            y: Some(self.state.pos[1]),
            z: Some(self.state.pos[2]),
            e: Some(self.state.e),
            f: Some(self.state.f),
        };
        let mut next = prev.apply(&cur);

        let dx = next.x.unwrap_or(0.0) - prev.x.unwrap_or(0.0);
        let dy = next.y.unwrap_or(0.0) - prev.y.unwrap_or(0.0);
        let dz = next.z.unwrap_or(0.0) - prev.z.unwrap_or(0.0);
        let de = next.e.unwrap_or(0.0) - prev.e.unwrap_or(0.0);

        let diff = Vector3::new(dx, dy, dz);

        let len = diff.magnitude();
        if len < std::f32::EPSILON {
            return (None, 0f32);
        }

        let step_len = next.f.unwrap_or(1800.0) / 60.0 * dt;
        if step_len >= len {
            // no need to split
            self.state.handle_gcode(next);
            return (None, dt * len / step_len);
        }

        let dx = dx * step_len / len;
        let dy = dy * step_len / len;
        let dz = dz * step_len / len;
        let de = de * step_len / len;

        next.x = Some(prev.x.unwrap_or(0.0) + dx);
        next.y = Some(prev.y.unwrap_or(0.0) + dy);
        next.z = Some(prev.z.unwrap_or(0.0) + dz);
        next.e = Some(prev.e.unwrap_or(0.0) + de);
        self.state.handle_gcode(next);

        (Some(cur), dt)
    }

    fn step0(&mut self, dt: f32) -> (bool, f32) {
        match self.pendings.pop() {
            Some((line, GCode1::Coord(cur))) => {
                if cur.major == 92 {
                    if let Some(x) = cur.x {
                        self.state.pos[0] = x;
                    }
                    if let Some(y) = cur.y {
                        self.state.pos[1] = y;
                    }
                    if let Some(z) = cur.z {
                        self.state.pos[2] = z;
                    }
                    if let Some(e) = cur.e {
                        self.state.e = e;
                    }

                    (false, 0.0)
                } else if [0, 1].contains(&cur.major) {
                    match self.g_0_1(cur, dt) {
                        (Some(next), step_dt) => {
                            self.pendings.push((line, GCode1::Coord(next)));
                            (false, step_dt)
                        }
                        (None, step_dt) => (false, step_dt),
                    }
                } else {
                    (false, 0f32)
                }
            }
            Some((_, GCode1::Layer(layer_idx))) => {
                info!("layer {}", layer_idx);
                /*
                if layer_idx > 0 && layer_idx % 10 == 0 {
                    let postfix = format!("{:03}", layer_idx);
                    self.state.export(out_filename, &postfix, true).unwrap();
                }
                */
                (false, 0.0)
            }
            Some(_) => (false, 0.0),
            None => (true, 0.0),
        }
    }
}

type FFIVoxel = ChunkedVoxel;

mod wrapper {
    use super::*;

    pub struct FFIRunner<V: StreamingVoxel> {
        pub runner: ExtrudeRunner<V>,
        pub buf: Vec<u8>,
    }

    impl<V: StreamingVoxel> FFIRunner<V> {
        pub fn new(runner: ExtrudeRunner<V>) -> Self {
            Self {
                runner,
                buf: vec![],
            }
        }

        pub fn step(&mut self, dt: f32) -> u64 {
            if self.runner.step(dt) {
                return 0;
            }
            self.buf.clear();

            let _ = self.runner.state.params.write(&mut self.buf);
            let _ = self.runner.state.mv.write_dirty(&mut self.buf);
            self.buf.len() as u64
        }

        pub fn set_write_options(&mut self, options: WriteOptions) {
            self.runner.state.mv.set_options(options);
        }
    }

    pub type RunnerWrapper = Arc<RwLock<Option<FFIRunner<FFIVoxel>>>>;

    pub fn with_wrapper<F>(ptr: usize, f: F)
    where
        F: FnOnce(&mut FFIRunner<FFIVoxel>),
    {
        let data: RunnerWrapper = unsafe { Arc::from_raw(ptr as *mut _) };
        {
            let mut guard = data.write().unwrap();
            if let Some(data) = guard.as_mut() {
                f(data);
            }
        }
        std::mem::forget(data);
    }

    pub fn from_utf16(ptr: *const u16, len: u32) -> Option<String> {
        if ptr.is_null() || len == 0 {
            return None;
        }
        let data: &[u16] = unsafe { std::slice::from_raw_parts(ptr, len as usize) };
        String::from_utf16(data).ok()
    }
}

use wrapper::*;

#[no_mangle]
pub unsafe extern "C" fn runner_new(filename_ptr: *const u16, filename_len: u32) -> *const u8 {
    let filename = from_utf16(filename_ptr, filename_len).unwrap();

    let parsed = match parse_gcode(&filename) {
        Ok(parsed) => parsed,
        Err(_e) => {
            return std::ptr::null();
        }
    };
    let runner = FFIRunner::new(ExtrudeRunner::<FFIVoxel>::new(parsed));

    let wrapper: RunnerWrapper = Arc::new(RwLock::new(Some(runner)));
    let ptr = RunnerWrapper::into_raw(wrapper);
    std::mem::transmute(ptr)
}

#[no_mangle]
pub unsafe extern "C" fn runner_delete(ptr: *const u8) {
    let data: RunnerWrapper = unsafe { Arc::from_raw(ptr as *mut _) };
    drop(data);
}

#[no_mangle]
pub unsafe extern "C" fn runner_step(ptr: *const u8, dt: f32, pos: *mut f32) -> u64 {
    let mut ret = 0u64;
    with_wrapper(ptr as usize, |runner| {
        ret = runner.step(dt);

        let dst: &mut [f32] = std::slice::from_raw_parts_mut(pos, 3);
        let pos = &runner.runner.pos();
        dst[0] = pos[0];
        dst[1] = pos[1];
        dst[2] = pos[2];
    });
    ret
}

#[no_mangle]
pub unsafe extern "C" fn runner_retrieve(ptr: *const u8, buf: *mut u8, len: u64) {
    with_wrapper(ptr as usize, |runner| {
        if runner.buf.len() != len as usize {
            return;
        }
        let dst: &mut [u8] = std::slice::from_raw_parts_mut(buf, len as usize);
        dst.copy_from_slice(&runner.buf);
    });
}

#[no_mangle]
pub unsafe extern "C" fn runner_set_params(ptr: *const u8, unit: f32) {
    with_wrapper(ptr as usize, |runner| {
        runner.runner.state.params.unit = unit;
    });
}

#[no_mangle]
pub unsafe extern "C" fn runner_set_write_options(ptr: *const u8, flags: u32) {
    with_wrapper(ptr as usize, |runner| {
        let options = match flags {
            0 => WriteOptions::None,
            1 => WriteOptions::Simplify,
            _ => {
                return;
            }
        };
        runner.set_write_options(options);
    });
}

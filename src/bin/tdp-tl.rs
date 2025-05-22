use anyhow::Result;
use argh::FromArgs;
use log::*;
use simple_stopwatch::Stopwatch;
use std::rc::Rc;
use tdp_tl::*;

#[derive(FromArgs)]
/// toplevel
struct TopLevel {
    #[argh(subcommand)]
    nested: SubCommandEnum,
}

#[derive(FromArgs, PartialEq, Debug)]
#[argh(subcommand)]
enum SubCommandEnum {
    DemoSphereFrames(DemoSphereFrames),
    DemoSphere(DemoSphere),
    DemoExtrude(DemoExtrude),
    Gcode(SubCommandGcode),
    GcodeLayers(SubCommandGcodeLayers),
}

#[derive(FromArgs, PartialEq, Debug)]
/// sphere frames
#[argh(subcommand, name = "demo-sphere-frames")]
struct DemoSphereFrames {
    /// const-z mode
    #[argh(option)]
    constz: bool,

    /// dryrun; do not render
    #[argh(option)]
    dryrun: bool,

    /// output directory
    #[argh(option)]
    outdir: String,
}

#[derive(FromArgs, PartialEq, Debug)]
/// sphere frames
#[argh(subcommand, name = "demo-sphere")]
struct DemoSphere {
    /// bruteforce
    #[argh(option)]
    bruteforce: bool,

    /// shell-only
    #[argh(option)]
    shell: bool,

    /// output filename
    #[argh(option)]
    out: String,
}

#[derive(FromArgs, PartialEq, Debug)]
/// demo: extrude
#[argh(subcommand, name = "demo-extrude")]
struct DemoExtrude {
    /// output filename
    #[argh(option)]
    out: String,
}

#[derive(FromArgs, PartialEq, Debug)]
/// gcode to obj
#[argh(subcommand, name = "gcode")]
struct SubCommandGcode {
    /// input filename
    #[argh(option)]
    gcode: String,

    /// output filename
    #[argh(option)]
    out: String,

    /// target number of layers
    #[argh(option)]
    layer: Option<usize>,
}

#[derive(FromArgs, PartialEq, Debug)]
/// gcode layers to obj
#[argh(subcommand, name = "gcode-layers")]
struct SubCommandGcodeLayers {
    /// input filename
    #[argh(option)]
    gcode: String,

    /// output directory
    #[argh(option)]
    outdir: String,

    /// use rangeset data structure
    #[argh(switch)]
    rangeset: bool,

    /// use svo data structure
    #[argh(switch)]
    svo: bool,

    /// chunked
    #[argh(switch)]
    chunked: bool,

    /// lod
    #[argh(switch)]
    lod: bool,

    /// iso
    #[argh(switch)]
    iso: bool,

    /// fsn
    #[argh(switch)]
    fsn: bool,

    /// fsn
    #[argh(switch)]
    vdb: bool,

    /// glb
    #[argh(switch)]
    glb: bool,
}

const SIZE: i32 = 100i32;
fn test(x: i32, y: i32, z: i32) -> bool {
    return x * x + y * y + z * z < SIZE * SIZE;
}

fn generate_brute_force() -> Vec<Rc<Model>> {
    let mut m = Model::default();

    for z in -SIZE..=SIZE {
        for y in -SIZE..=SIZE {
            for x in -SIZE..=SIZE {
                if test(x, y, z) {
                    m.add_cube([x, y, z].into());
                }
            }
        }
    }

    vec![Rc::new(m)]
}

fn generate_shell() -> Vec<Rc<Model>> {
    let mut m = Model::default();

    const NEIGHBORS: [[i32; 3]; 6] = [
        [1, 0, 0],
        [-1, 0, 0],
        [0, 1, 0],
        [0, -1, 0],
        [0, 0, 1],
        [0, 0, -1],
    ];

    fn emit(x: i32, y: i32, z: i32) -> bool {
        let r0 = test(x, y, z);
        for [dx, dy, dz] in &NEIGHBORS {
            let r1 = test(x + dx, y + dy, z + dz);
            if r0 != r1 {
                return true;
            }
        }
        return false;
    }

    for z in -SIZE..=SIZE {
        for y in -SIZE..=SIZE {
            for x in -SIZE..=SIZE {
                if emit(x, y, z) {
                    m.add_cube([x, y, z].into());
                }
            }
        }
    }

    vec![Rc::new(m)]
}

fn generate_face_only() -> Vec<Rc<Model>> {
    let mut mv = MonotonicVoxel::default();

    for z in -SIZE..=SIZE {
        for y in -SIZE..=SIZE {
            for x in -SIZE..=SIZE {
                if test(x, y, z) {
                    mv.add([x, y, z].into());
                }
            }
        }
    }
    mv.to_model()
}

fn generate_frames_constz<V: Voxel>(outdir: &String) -> Result<()> {
    let mut mv = V::default();

    let mut idx = 0;
    for z in -SIZE..=SIZE {
        for y in -SIZE..=SIZE {
            for x in -SIZE..=SIZE {
                if test(x, y, z) {
                    mv.add([x, y, z].into());
                }
            }
        }
        if z < 0 {
            continue;
        }

        let model = mv.to_model();
        let filename = format!("{}/out_{:.2}.obj", outdir, idx);
        model_serialize(&model, &filename, [0f32; 3], 1f32)?;
        idx += 1;
    }
    Ok(())
}

fn generate_extrude(out: &str) -> Result<()> {
    let mut mv = MonotonicVoxel::default();

    // unit: 0.02mm, layer thickness: 0.2mm, nozzle size: 0.4mm
    // 20mm

    let extrude_per_dist = 200;
    let dist_per_step = 5;
    let dist = 100;
    for step in 0..(dist / dist_per_step) {
        let p = VoxelIdx::from([step * dist_per_step, 0, 0]);
        extrude_at(
            &mut mv,
            -5..5,
            &[p],
            (extrude_per_dist * dist_per_step) as usize,
        );
    }

    let model = mv.to_model();
    model_serialize(&model, out, [0f32; 3], 1f32)
}

fn generate_frames<V: Voxel>(outdir: &str, render: bool) -> Result<()> {
    let mut mv = V::default();

    let mut count: usize = 0;

    let mut dt_render = 0f32;

    let sw = Stopwatch::start_new();
    let mut idx = 0;
    for z in -SIZE..=SIZE {
        for y in -SIZE..=SIZE {
            for x in -SIZE..=SIZE {
                if test(x, y, z) {
                    mv.add([x, y, z].into());

                    count += 1;
                    if render && count % 20000 == 0 {
                        info!("render={:?}", (x, y, z));
                        let sw = Stopwatch::start_new();

                        let model = mv.to_model();
                        let filename = format!("{}/out_{:.2}.obj", outdir, idx);
                        model_serialize(&model, &filename, [0f32; 3], 1f32)?;
                        idx += 1;

                        dt_render += sw.ms();
                    }
                }
            }
        }
    }

    let elapsed = sw.ms();
    info!(
        "voxel construction: total={:.2}ms, render={:.2}ms, blocks={}, bps={}",
        elapsed,
        dt_render,
        count,
        count * 1000 / elapsed as usize
    );

    Ok(())
}

fn main() -> Result<()> {
    env_logger::init();

    let opt: TopLevel = argh::from_env();

    match opt.nested {
        SubCommandEnum::DemoSphereFrames(opt) => {
            if opt.constz {
                generate_frames_constz::<MonotonicVoxel>(&opt.outdir)
            } else {
                if false {
                    generate_frames::<SVOVoxel>(&opt.outdir, !opt.dryrun).ok();
                    generate_frames::<RangeSetVoxel>(&opt.outdir, !opt.dryrun).ok();
                }
                generate_frames::<MonotonicVoxel>(&opt.outdir, !opt.dryrun).ok();
                Ok(())
            }
        }

        SubCommandEnum::DemoSphere(opt) => {
            let model = if opt.bruteforce {
                generate_brute_force()
            } else if opt.shell {
                generate_shell()
            } else {
                generate_face_only()
            };

            model_serialize(&model, &opt.out, [0f32; 3], 1f32)?;
            Ok(())
        }

        SubCommandEnum::DemoExtrude(opt) => generate_extrude(&opt.out),

        SubCommandEnum::Gcode(opt) => {
            let layer = opt.layer.unwrap_or(std::usize::MAX);
            generate_gcode::<MonotonicVoxel>(&opt.gcode, &opt.out, layer, false, false)
        }

        SubCommandEnum::GcodeLayers(opt) => {
            let layer = std::usize::MAX;
            if opt.rangeset {
                generate_gcode::<RangeSetVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.svo {
                generate_gcode::<SVOVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.chunked {
                generate_gcode::<ChunkedVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.lod {
                generate_gcode::<LodVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.iso {
                generate_gcode::<IsoVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.fsn {
                generate_gcode::<FSNVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else if opt.vdb {
                generate_gcode::<VDBVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            } else {
                generate_gcode::<MonotonicVoxel>(&opt.gcode, &opt.outdir, layer, true, opt.glb)
            }
        }
    }
}

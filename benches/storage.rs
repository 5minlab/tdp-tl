use criterion::{black_box, criterion_group, criterion_main, Criterion};
use tdp_tl::*;

const GCODE_PATH: &str = "demo/KK_xyzCalibration_cube.gcode";
const LAYERS: usize = 2;

fn prep_gcode() -> Vec<(usize, GCode1)> {
    parse_gcode(GCODE_PATH).expect("load gcode")
}

fn slice_layers(data: &[(usize, GCode1)]) -> &[(usize, GCode1)] {
    let mut idx = data.len();
    for (i, (_, item)) in data.iter().enumerate() {
        if let GCode1::Layer(layer) = item {
            if *layer >= LAYERS {
                idx = i;
                break;
            }
        }
    }
    &data[..idx]
}

fn run_layers<V: Voxel + Default>(data: &[(usize, GCode1)]) -> usize {
    simulate_gcode_layers::<V>(data, LAYERS)
}

fn benchmark_voxels(c: &mut Criterion) {
    let parsed = prep_gcode();
    let data = slice_layers(&parsed);

    let mut group = c.benchmark_group("voxel_storage");
    group.bench_function("Monotonic", |b| b.iter(|| black_box(run_layers::<MonotonicVoxel>(data))));
    group.bench_function("RangeSet", |b| b.iter(|| black_box(run_layers::<RangeSetVoxel>(data))));
    group.bench_function("Chunked", |b| b.iter(|| black_box(run_layers::<ChunkedVoxel>(data))));
    group.bench_function("SVO", |b| b.iter(|| black_box(run_layers::<SVOVoxel>(data))));
    group.bench_function("LOD", |b| b.iter(|| black_box(run_layers::<LodVoxel>(data))));
    group.bench_function("ISO", |b| b.iter(|| black_box(run_layers::<IsoVoxel>(data))));
    group.bench_function("FSN", |b| b.iter(|| black_box(run_layers::<FSNVoxel>(data))));
    group.finish();
}

criterion_group!(benches, benchmark_voxels);
criterion_main!(benches);


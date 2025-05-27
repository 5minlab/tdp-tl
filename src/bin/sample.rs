use tdp_tl::*;

fn main() {
    let mut cell = BGMCell::default();

    for i in 0..32 {
        for j in 0..32 {
            for k in 0..32 {
                // 3d checker pattern
                if (i + j + k) % 2 == 0 {
                    cell.set(i, j, k);
                }
            }
        }
    }

    let mut v = Vec::<VoxelIdx>::new();

    for x in 0..CELL_SIZE {
        for y in 0..CELL_SIZE {
            for z in 0..CELL_SIZE {
                let val = cell.get(x, y, z);
                let sum = cell.neighbors_sum(VoxelIdx::new([x as i32, y as i32, z as i32]));
                if (!val && sum > 3) || (val && sum < 3) {
                    v.push(VoxelIdx::new([x as i32, y as i32, z as i32]));
                }
            }
        }
    }

    while let Some(coord) = v.pop() {
        let [x, y, z] = cell_idx(coord);

        let val = cell.get(x, y, z);
        let sum = cell.neighbors_sum(coord);
        if !val && sum > 3 {
            cell.set(x, y, z);
        }
        /*else if val && sum < 3 {
            cell.clear(x, y, z);
        } */
        else {
            continue;
        }

        for n in cell_neighbors(coord) {
            v.push(n);
        }
    }

    // LOD while preserving the shape
    //

    eprintln!("{:?}", cell);
}

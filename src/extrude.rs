use super::*;
use std::ops::Range;

pub fn line_cells(pos0: VoxelIdx, pos1: VoxelIdx, cells: &mut Vec<VoxelIdx>) {
    let dx = (pos1[0] - pos0[0]) as f32;
    let dy = (pos1[1] - pos0[1]) as f32;
    let dz = (pos1[2] - pos0[2]) as f32;

    let steps = ((dx * dx + dy * dy + dz * dz).sqrt() * 2.0) as usize;

    let mut cur = pos0;
    cells.push(cur);
    for i in 0..steps {
        let x = pos0[0] as f32 + dx * i as f32 / steps as f32;
        let y = pos0[1] as f32 + dy * i as f32 / steps as f32;
        let z = pos0[2] as f32 + dz * i as f32 / steps as f32;
        let next = VoxelIdx::new([x, y, z].map(|v| v.round() as i32));
        if next == cur {
            continue;
        }

        cells.push(next);
        cur = next;
    }
}

pub fn extrude_at<V: Voxel>(
    v: &mut V,
    zrange: Range<i32>,
    max_dist: usize,
    cells: &[VoxelIdx],
    n: usize,
) -> usize {
    if true {
        extrude_at_queue(v, zrange, max_dist, cells, n)
    } else {
        extrude_at_deque(v, zrange, max_dist, cells, n)
    }
}

pub fn extrude_at_queue<V: Voxel>(
    v: &mut V,
    zrange: Range<i32>,
    max_dist: usize,
    cells: &[VoxelIdx],
    n: usize,
) -> usize {
    use std::collections::BinaryHeap;

    if n == 0 {
        return 0;
    }

    let mut extrudeed = 0;

    #[derive(Clone, Copy, Ord, PartialEq, Eq, Debug)]
    struct HeapItem {
        dist: usize,
        src: VoxelIdx,
        pos: VoxelIdx,
    }
    impl std::cmp::PartialOrd for HeapItem {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(other.dist.cmp(&self.dist))
        }
    }

    let mut candidates = BinaryHeap::new();
    let mut visited = ChunkedVoxel::default();

    for pos in cells {
        let pos = *pos;
        candidates.push(HeapItem {
            dist: 0,
            src: pos,
            pos,
        });
    }

    while let Some(HeapItem {
        dist: _dist,
        src,
        pos,
    }) = candidates.pop()
    {
        if !visited.add(pos) {
            continue;
        }

        if v.add(pos) {
            extrudeed += 1;
            if n == extrudeed {
                break;
            }
        }

        let directions = [
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
            [0, 0, 1],
            [0, 0, -1],
        ];

        for dir in directions {
            let next: VoxelIdx = pos + dir.into();
            if !zrange.contains(&next[2]) {
                continue;
            }
            if visited.occupied(next) {
                continue;
            }

            let delta = src - next;
            let dist = delta.magnitude_squared();
            if dist > max_dist {
                continue;
            }
            candidates.push(HeapItem {
                dist,
                src,
                pos: next,
            });
        }
    }

    extrudeed
}

pub fn extrude_at_deque<V: Voxel>(
    v: &mut V,
    zrange: Range<i32>,
    max_dist: usize,
    cells: &[VoxelIdx],
    n: usize,
) -> usize {
    use std::collections::VecDeque;

    if n == 0 {
        return 0;
    }

    let mut extrudeed = 0;

    #[derive(Clone, Copy, PartialEq, Eq, Debug)]
    struct HeapItem {
        pos: VoxelIdx,
        depth: u16,
    }

    let mut candidates = VecDeque::with_capacity(1024 * 8);
    let mut visited = ChunkedVoxel::default();

    for pos in cells {
        visited.add(*pos);
        candidates.push_back(HeapItem {
            pos: *pos,
            depth: max_dist as u16,
        });
    }

    while let Some(HeapItem { pos, depth }) = candidates.pop_front() {
        if v.add(pos) {
            extrudeed += 1;
            if n == extrudeed {
                break;
            }
        }

        if depth == 0 {
            continue;
        }

        let directions = [
            [0, 0, 1],
            [0, 0, -1],
            [1, 0, 0],
            [-1, 0, 0],
            [0, 1, 0],
            [0, -1, 0],
        ];

        for dir in directions {
            let next: VoxelIdx = pos + dir.into();
            if !zrange.contains(&next[2]) {
                continue;
            }
            if !visited.add(next) {
                continue;
            }

            candidates.push_back(HeapItem {
                pos: next,
                depth: depth - 1,
            });
        }
    }

    extrudeed
}

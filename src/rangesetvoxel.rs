use super::{BoundingBox, Model, Voxel, VoxelIdx};
use rangemap::RangeSet;

#[derive(Default)]
pub struct RangeSetVoxel {
    ranges: RangeSet<VoxelIdx>,
    bb: BoundingBox,
}

impl Voxel for RangeSetVoxel {
    fn ranges(&self) -> usize {
        self.ranges.iter().count()
    }

    fn bounding_box(&self) -> &BoundingBox {
        &self.bb
    }

    fn occupied(&self, coord: VoxelIdx) -> bool {
        self.ranges.contains(&coord)
    }

    fn add(&mut self, coord: VoxelIdx) -> bool {
        if self.occupied(coord) {
            return false;
        }

        let end = coord + VoxelIdx::new([0, 0, 1]);
        self.ranges.insert(coord..end);
        self.bb.add(coord);
        true
    }

    fn to_model(&self) -> Model {
        let mut model = Model::default();

        let mut ranges_t: RangeSet<VoxelIdx> = RangeSet::new();

        for range in self.ranges.iter() {
            assert_eq!(range.start.xy(), range.end.xy());
            let x = range.start[0];
            let y = range.start[1];

            let z0 = range.start[2];
            let z1 = range.end[2];

            {
                let t_start = VoxelIdx::new([z1, x, y]);
                let r_end = VoxelIdx::new([z1, x, y + 1]);
                ranges_t.insert(t_start..r_end);
            }

            let faces = [
                ([1, 0], [1, 1, 1], [0, -1, -1]),
                ([-1, 0], [0, 0, 0], [0, 1, 1]),
                ([0, 1], [1, 1, 1], [-1, 0, -1]),
                ([0, -1], [0, 0, 0], [1, 0, 1]),
            ];

            for ([dx, dy], offset, dir) in faces {
                let mut segments = vec![];
                let mut z_start = z0;
                for z in z0..z1 {
                    if self.occupied([x + dx, y + dy, z].into()) {
                        if z_start != z {
                            segments.push(z_start..z);
                        }
                        z_start = z + 1;
                    }
                }
                if z_start < z1 {
                    segments.push(z_start..z1);
                }

                for segment in segments {
                    let z0 = segment.start;
                    let z1 = segment.end;
                    let dz = z1 - z0;

                    model.add_face(
                        [x + offset[0], y + offset[1], z0 + offset[2] * dz].into(),
                        [dir[0], dir[1], dir[2] * dz].into(),
                    );
                }
            }
        }

        for range in ranges_t.iter() {
            assert_eq!(range.start.xy(), range.end.xy());

            let z = range.start[0];
            let x = range.start[1];
            let y0 = range.start[2];
            let y1 = range.end[2];
            let dy = y1 - y0;

            let up = VoxelIdx::from([1, dy, 0]);
            model.add_face([x, y0, z].into(), up);
        }

        model
    }
}

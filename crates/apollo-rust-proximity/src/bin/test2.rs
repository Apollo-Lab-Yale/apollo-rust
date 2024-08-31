use parry3d_f64::bounding_volume::{SimdAabb};
use parry3d_f64::math::{SimdBool};
use parry3d_f64::partitioning::{Qbvh, QbvhUpdateWorkspace, SimdSimultaneousVisitor, SimdSimultaneousVisitStatus};
use parry3d_f64::shape::{Ball};
use apollo_rust_proximity::offset_shape::OffsetShape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

pub struct Visitor {
    pub intersections: Vec<(usize, usize)>
}
impl SimdSimultaneousVisitor<usize, usize, SimdAabb> for Visitor {
    fn visit(&mut self, left_bv: &SimdAabb, left_data: Option<[Option<&usize>; 4]>, right_bv: &SimdAabb, right_data: Option<[Option<&usize>; 4]>) -> SimdSimultaneousVisitStatus {
        let intersects = left_bv.intersects(right_bv).0;
        let mut out = [SimdBool::ZERO; 4];
        for (i, intersect) in intersects.iter().enumerate() {
            if *intersect {
                out[i] = SimdBool::ONE;
                if let (Some(left_data), Some(right_data)) = (left_data, right_data) {
                    if let (Some(data_left), Some(data_right)) = (left_data[i], right_data[i]) {
                        self.intersections.push((*data_left, *data_right));

                        // Check cross-pairs manually if the indices are different
                        for (j, intersect_inner) in intersects.iter().enumerate() {
                            if i != j && *intersect_inner {
                                self.intersections.push((*data_left, *right_data[j].unwrap()));
                                self.intersections.push((*data_right, *left_data[j].unwrap()));
                            }
                        }
                    }
                }
            }
        }

        SimdSimultaneousVisitStatus::MaybeContinue(out)
    }
}


fn main() {
    let mut qbvh = Qbvh::<usize>::new();

    let shapes = vec![OffsetShape::new(Ball::new(0.2), None), OffsetShape::new(Ball::new(0.2), None)];
    let poses = vec![ISE3q::identity(), ISE3q::identity()];

    qbvh.pre_update_or_insert(0);
    qbvh.pre_update_or_insert(1);

    let mut workspace = QbvhUpdateWorkspace::default();
    qbvh.refit(0.0, &mut workspace, |data| {
        let pose = shapes[*data].get_transform(&poses[*data]);
        shapes[*data].shape().compute_aabb(&pose.as_ref().0)
    });

    let qbvh2 = qbvh.clone();

    let mut visitor = Visitor { intersections: vec![] };

    qbvh.traverse_bvtt(&qbvh2, &mut visitor);
    println!("{:?}", visitor.intersections);
}
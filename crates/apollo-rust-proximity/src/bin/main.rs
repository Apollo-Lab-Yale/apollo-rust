use parry3d_f64::query::{contact};
use parry3d_f64::shape::Cuboid;
use apollo_rust_proximity::offset_shape::OffsetShape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;

fn main() {
    let shape_a = OffsetShape::new(Cuboid::new(V3::new(5., 5., 5.)), None);
    let shape_b = OffsetShape::new(Cuboid::new(V3::new(5., 5., 5.)), None);

    let pose_a = ISE3q::new_random();
    let pose_b = ISE3q::new_random();

    let result = contact(&pose_a.0, &**shape_a.shape(), &pose_b.0, &**shape_b.shape(), -f64::INFINITY);
    println!("{:?}", result);
}


use apollo_rust_proximity_parry::offset_shape::OffsetShape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use parry3d_f64::shape::Cuboid;

fn main() {
    let shape_a = OffsetShape::new(Cuboid::new(V3::new(10.1, 10.1, 10.1)), None);
    let shape_b = OffsetShape::new(Cuboid::new(V3::new(10.1, 10.1, 10.1)), None);

    let pose_a = ISE3q::new_random();
    let pose_b = ISE3q::new_random();

    let res = shape_a.contact(&pose_a, &shape_b, &pose_b, 1000.0).unwrap();
    println!("{:?}", res);

    println!("{:?}", (res.point1 - res.point2).norm());
}

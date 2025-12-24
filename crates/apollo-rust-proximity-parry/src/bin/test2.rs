use apollo_rust_proximity_parry::offset_shape::OffsetShape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use parry3d_f64::bounding_volume::Aabb;
use parry3d_f64::partitioning::{Bvh, BvhBuildStrategy};
use parry3d_f64::shape::Ball;

fn main() {
    let shapes = vec![
        OffsetShape::new(Ball::new(0.2), None),
        OffsetShape::new(Ball::new(0.2), None),
    ];
    let poses = vec![ISE3q::identity(), ISE3q::identity()];

    let aabbs: Vec<Aabb> = shapes
        .iter()
        .zip(&poses)
        .map(|(s, p)| {
            let pose = s.get_transform(p);
            s.shape().compute_aabb(&pose.as_ref().0)
        })
        .collect();

    let bvh = Bvh::from_leaves(BvhBuildStrategy::default(), &aabbs);
    let bvh2 = bvh.clone();

    let collisions: Vec<(u32, u32)> = bvh.leaf_pairs(&bvh2, |a, b| a.intersects(b)).collect();

    println!("{:?}", collisions);
}

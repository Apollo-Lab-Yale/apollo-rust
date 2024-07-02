use std::time::Instant;
use parry3d_f64::shape::{Ball, Cuboid, Shape};
use apollo_proximity::pairwise_group_queries::pairwise_group_query_contact;
use apollo_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_spatial::vectors::V3;

fn main() {
    let a = Ball::new(2.0);
    let b = Cuboid::new(V3::new(1.,2.,3.));

    let shapes_a: Vec<&dyn Shape> = vec![&a, &b];

    let pa = vec![ ISE3q::new(I3::new_random_with_range(0.0, 10.0)), ISE3q::new(I3::new_random_with_range(0.0, 10.0)) ];

    let now = Instant::now();
    for _ in 0..1000 {
        pairwise_group_query_contact(&shapes_a, &pa, &shapes_a, &pa, true, None, false, 100.0);
    }
    println!("{:?}", now.elapsed());

    // res.iter().for_each(|x| println!("{:?}", x));
}


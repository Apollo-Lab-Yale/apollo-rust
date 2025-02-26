use rand::Rng;
use parry3d_f64;
use nalgebra::{Isometry3, UnitQuaternion, Translation3};
use parry3d_f64::shape::{Cuboid as ParryCuboid, Ball as ParryBall};
use parry3d_f64::query::distance as parry_distance;
use apollo_rust_proximity::{gjk_distance, Cuboid, Sphere};
use apollo_rust_spatial::vectors::V3;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;

fn random_pose() -> LieGroupISE3q {
    LieGroupISE3q::new_random()
}

fn random_cuboid() -> Cuboid {
    let mut rng = rand::thread_rng();
        // Create a Cuboid with random half extents in [0.1, 10.0].
    let x = rng.gen_range(0.5..1.0);
    let y = rng.gen_range(0.5..1.0);
    let z = rng.gen_range(0.5..1.0);
    Cuboid::new(x, y, z)
}


fn main() {
    let iterations = 1000;
    let tolerance = 1e-4;

    for i in 0..iterations {

        let s1 = random_cuboid();
        let s2 = random_cuboid();
        let p1 = random_pose();
        let p2 = random_pose();

        let dist = gjk_distance(&s1, &p1, &s2, &p2);

        let s1_parry = ParryCuboid::new(s1.half_extents.into());
        let s2_parry = ParryCuboid::new(s2.half_extents.into());

        let mut dist0 = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap().max(0.0);
        if dist0>0.0 {dist0=1.0;}
        let diff = (dist - dist0).abs();
        if diff > tolerance {
            eprintln!(
                "Iteration {}: Distance mismatch: my = {} vs parry = {} (diff = {})",
                i, dist, dist0, diff
            );
        }
    }
}

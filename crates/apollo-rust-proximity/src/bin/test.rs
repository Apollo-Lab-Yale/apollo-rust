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
const MIN_HALF_EXTENDS: f64 = 0.5;
const MAX_HALF_EXTENDS: f64 = 2.0;
fn random_cuboid() -> Cuboid {
    let mut rng = rand::thread_rng();
        // Create a Cuboid with random half extents in [0.1, 10.0].
    let x = rng.gen_range(MIN_HALF_EXTENDS..MAX_HALF_EXTENDS);
    let y =  rng.gen_range(MIN_HALF_EXTENDS..MAX_HALF_EXTENDS);
    let z =  rng.gen_range(MIN_HALF_EXTENDS..MAX_HALF_EXTENDS);
    Cuboid::new(x, y, z)
}


fn main() {
    let iterations = 1000;
    let tolerance = 1e-4;
    let check_distance = true;
    for i in 0..iterations {
        println!("Iter {}",i);
        let s1 = random_cuboid();
        let s2 = random_cuboid();
        let p1 = random_pose();
        let p2 = random_pose();

        let dist = gjk_distance(&s1, &p1, &s2, &p2);
        let collided = if dist>0.0 {false} else {true};

        let s1_parry = ParryCuboid::new(s1.half_extents.into());
        let s2_parry = ParryCuboid::new(s2.half_extents.into());

        let dist0 = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap();
        let collided0 = if dist0>0.0 {false} else {true};
        if collided0&&!check_distance {println!("In iter {}, collides", i)}
        // check
        if collided!=collided0 {
            eprintln!(
                "Iteration {}: Collision check mismatch: my_collided = {} vs parry_collided = {}",
                i, collided, collided0
            );
        }
        if check_distance {
            let diff = (dist - dist0).abs();
            if diff > tolerance {
                eprintln!(
                    "Iteration {}: Distance mismatch: my = {} vs parry = {} (diff = {})",
                    i, dist, dist0, diff
                );
            }
        }
    }
}

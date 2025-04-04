use rand::Rng;
use parry3d_f64;
use std::time::{Duration, Instant};
use nalgebra::{Isometry3, UnitQuaternion, Translation3};
use parry3d_f64::shape::{Cuboid as ParryCuboid, Ball as ParryBall, ConvexPolyhedron as ParryConvexPolyhedron};
use parry3d_f64::math::Point as ParryPoint;
use parry3d_f64::query::distance as parry_distance;
use parry3d_f64::query::contact as parry_contact;
use apollo_rust_proximity::{gjk_contact, Cuboid, Sphere};
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

fn cuboid_to_polyhedron(cuboid: &Cuboid) -> ParryConvexPolyhedron {
    // Extract the half-extents.
    let hx = cuboid.half_extents.x;
    let hy = cuboid.half_extents.y;
    let hz = cuboid.half_extents.z;

    // Compute the 8 vertices of the cuboid. The cuboid is assumed centered at the origin.
    let vertices = vec![
        ParryPoint::new( hx,  hy,  hz), // 0
        ParryPoint::new(-hx,  hy,  hz), // 1
        ParryPoint::new(-hx, -hy,  hz), // 2
        ParryPoint::new( hx, -hy,  hz), // 3
        ParryPoint::new( hx,  hy, -hz), // 4
        ParryPoint::new(-hx,  hy, -hz), // 5
        ParryPoint::new(-hx, -hy, -hz), // 6
        ParryPoint::new( hx, -hy, -hz), // 7
    ];

    // Define the 6 faces of the cuboid.
    // Each face is defined by 4 vertex indices in counter-clockwise order.
    let quad_faces = vec![
        [0, 1, 2, 3], // Front face  (z positive)
        [4, 5, 6, 7], // Back face   (z negative)
        [0, 3, 7, 4], // Right face  (x positive)
        [1, 2, 6, 5], // Left face   (x negative)
        [0, 1, 5, 4], // Top face    (y positive)
        [3, 2, 6, 7], // Bottom face (y negative)
    ];

    let mut tri_faces = Vec::with_capacity(quad_faces.len() * 2);
    for quad in quad_faces {
        tri_faces.push([quad[0], quad[1], quad[2]]);
        tri_faces.push([quad[0], quad[2], quad[3]]);
    }
    ParryConvexPolyhedron::from_convex_mesh(vertices, &tri_faces).unwrap()
}

fn main() {
    let iterations = 10000;
    let tolerance = 1e-4;
    let check_distance = true;

    let mut my_non_contact_time = Duration::new(0, 0);
    let mut my_contact_time = Duration::new(0, 0);
    let mut parry_non_contact_time = Duration::new(0, 0);
    let mut parry_contact_time = Duration::new(0, 0);
    let mut contacts = 0;

    for i in 0..iterations {
        println!("Iter {}",i);
        let s1 = random_cuboid();
        let s2 = random_cuboid();
        let p1 = random_pose();
        let p2 = random_pose();

        let start = Instant::now();
        let (dir, dist) = gjk_contact(&s1, &p1, &s2, &p2);
        let elapsed = start.elapsed();
        let collided = if dist>0.0 {false} else {true};
        if collided {my_contact_time += elapsed; contacts+=1;} else {my_non_contact_time += elapsed;}
        //let s1_parry = ParryCuboid::new(s1.half_extents.into());
        //let s2_parry = ParryCuboid::new(s2.half_extents.into());
        let s1_parry= cuboid_to_polyhedron(&s1);
        let s2_parry= cuboid_to_polyhedron(&s2);
        let start = Instant::now();
        let mut dist0 = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap();
        let elapsed = start.elapsed();
        let collided0 = if dist0>0.0 {false} else {true};
        if collided0 {
            let start = Instant::now();
            let contact = parry_contact(&p1.0, &s1_parry, &p2.0, &s2_parry, 0.0).unwrap();
            let elapsed = start.elapsed();
            dist0 = contact.unwrap().dist;
            parry_contact_time += elapsed;}
        else {parry_non_contact_time += elapsed;}

        // check
        if collided!=collided0 {
            panic!(
                "Iteration {}: Collision check mismatch: my_collided = {} vs parry_collided = {}",
                i, collided, collided0
            );
        }
        if check_distance {
            let diff = (dist - dist0).abs();
            if diff > tolerance {
                panic!(
                    "Iteration {}: Distance mismatch: my = {} vs parry = {} (diff = {})",
                    i, dist, dist0, diff
                );
            }
        }
    }
    // profile
    println!("In {} tests, contacts happened {} times", iterations, contacts);
    println!("Non-contact (without EPA): my_time={:?}, parry_time={:?}", my_non_contact_time, parry_non_contact_time);
    println!("Contact (with EPA): my_time={:?}, parry_time={:?}", my_contact_time, parry_contact_time);
}

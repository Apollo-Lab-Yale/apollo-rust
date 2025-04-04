use std::time::{Duration, Instant};
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector4};
use apollo_rust_proximity::{gjk_contact, Cuboid, Sphere};
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::vectors::V3;
use parry3d_f64::shape::{Cuboid as ParryCuboid, Ball as ParryBall, ConvexPolyhedron as ParryConvexPolyhedron};
use parry3d_f64::math::Point as ParryPoint;
use parry3d_f64::query::distance as parry_distance;
use parry3d_f64::query::contact as parry_contact;

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
    let tolerance = 1e-4;
    let s1 = Cuboid::new( 1.0606908305142144 ,
                          0.650219314198895 ,
                          1.8177101125010569);
    let s2 =  Cuboid::new(  0.5122661185580153 ,
                            1.1620654900501135 ,
                            1.8490306812866195 );
    let mut p1 = LieGroupISE3q::identity();
    //p1.0.rotation = UnitQuaternion::new_normalize(Quaternion::new(0.1386963726538735,-0.7010145655139903, 0.1396139374142936, 0.6854559385033928));
    //p1.0.translation=Translation3::new(2.749148927243306, -0.634933474296135, 2.5467344103108234);
    let mut p2 = LieGroupISE3q::identity();
        let (dir, dist) = gjk_contact(&s1, &p1, &s2, &p2);
        let collided = if dist>0.0 {false} else {true};
        //let s1_parry = ParryCuboid::new(s1.half_extents.into());
        //let s2_parry = ParryCuboid::new(s2.half_extents.into());
        let s1_parry= cuboid_to_polyhedron(&s1);
        let s2_parry= cuboid_to_polyhedron(&s2);
        let mut dist0 = parry_distance(&p1.0, &s1_parry, &p2.0, &s2_parry).unwrap();
        let collided0 = if dist0>0.0 {false} else {true};
        if collided0 {
            let contact = parry_contact(&p1.0, &s1_parry, &p2.0, &s2_parry, 0.0).unwrap();
            dist0 = contact.unwrap().dist;}
            let diff = (dist - dist0).abs();//if !collided {(dist - dist0).abs()} else {dist - dist0};
                println!(
                    "my = {} vs parry = {} (diff = {}), cuboid1.half_extends={}, cuboid2.half_extends={}, p1={:?}, p2={:?}",
                    dist, dist0, diff,s1.half_extents, s2.half_extents,p1,p2
                );

}
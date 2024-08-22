use std::time::Instant;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let c = r.get_subdirectory("b1").to_chain_nalgebra();
    let mut p = c.get_self_proxima1(0.0, &V::new(&[0.0; 12]), LinkShapeMode::Full, LinkShapeRep::ConvexHull);
    let link_poses = c.fk(&V::new(&[0.001; 12]));
    let res = c.self_intersect_proxima(&mut p, &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res.ground_truth_checks.len());
    let link_poses = c.fk(&V::new(&[0.05; 12]));
    let res = c.self_intersect_proxima(&mut p, &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res.ground_truth_checks.len());
    let link_poses = c.fk(&V::new(&[0.1; 12]));
    let start = Instant::now();
    let res = c.self_intersect_proxima(&mut p, &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false);
    println!("{:?}", start.elapsed());
    println!("{:?}", res.ground_truth_checks.len());
    // let link_poses = c.fk(&V::new(&[0.5; 6]));
    // let res = c.self_intersect_proxima(&mut p, &ProximaBudget::AccuracyBudget(0.3), &link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull);
    // println!("{:?}", res);
    let start = Instant::now();
    let res = c.self_contact(&link_poses, LinkShapeMode::Full, LinkShapeRep::ConvexHull, false, 1000.);
    println!("{:?}", start.elapsed());
    println!("{:?}", res.num_ground_truth_checks);
}
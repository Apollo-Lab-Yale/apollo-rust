use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let c = r.get_subdirectory("b1").to_chain_nalgebra();
    let mut p = c.get_self_proxima1(0.0, &V::new(&[0.0; 12]), LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull);
    let link_poses = c.fk(&V::new(&[0.5; 12]));
    let res = c.self_intersect_proxima(&mut p, &link_poses, LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res);
}
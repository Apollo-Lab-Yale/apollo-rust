use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_proximity::double_group_queries::IntersectionFoundTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();

    let res = c.self_intersect_from_state(&V::new(&[0.0; 6]), LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull, false);
    println!("{:?}", res.intersection_found());
}
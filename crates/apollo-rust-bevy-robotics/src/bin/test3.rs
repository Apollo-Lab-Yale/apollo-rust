use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{ToChainNalgebra};
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();
    println!("{:?}", c.num_dofs());
    // let r = ResourcesRootDirectory::new_from_default_apollo_environments_directory();
    // let s2 = r.get_subdirectory("test");
    // let c2 = s2.to_chain_nalgebra();
    // c.bevy_proximity_vis();
    let cc = c.get_self_proxima1_cache(&V::new(&[0.0; 6]), LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull);
    println!("{:?}", cc);
}
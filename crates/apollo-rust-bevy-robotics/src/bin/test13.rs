use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{PreprocessForceBuildModulesTrait, ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();

    // println!("{:?}", c.fk(&V::new(&[0.0])));
    c.refine_link_shapes_skips_module();
}
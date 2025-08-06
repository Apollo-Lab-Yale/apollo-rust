use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{PreprocessForceBuildModulesTrait, ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("d1");
    let c = s.to_chain_nalgebra();
    c.bevy_display();
}
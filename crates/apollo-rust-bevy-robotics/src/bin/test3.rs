use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("robotiq_140").to_chain_nalgebra();
    c.bevy_display();
}
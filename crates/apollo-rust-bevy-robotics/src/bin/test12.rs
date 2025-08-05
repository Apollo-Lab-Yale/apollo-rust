use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ability_hand_left");

    let c = s.to_chain_nalgebra();
}
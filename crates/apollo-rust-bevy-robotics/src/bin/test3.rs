use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("xarm7");
    let c = s.to_chain_nalgebra();
    let r = ResourcesRootDirectory::new_from_default_apollo_environments_directory();
    let s2 = r.get_subdirectory("test");
    let c2 = s2.to_chain_nalgebra();
    c.bevy_two_chain_display(&c2);
}
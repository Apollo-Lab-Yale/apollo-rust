use std::path::PathBuf;
use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChain;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_environments_dir());
    let s = r.get_subdirectory("test");
    let c = s.to_chain();
    c.bevy_display(&PathBuf::new_from_default_apollo_bevy_assets_dir());
}
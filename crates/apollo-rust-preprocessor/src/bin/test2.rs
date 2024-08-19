use apollo_rust_preprocessor::{ResourcesRootDirectoryTrait};
use apollo_rust_robot_modules::ResourcesRootDirectory;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    r.preprocess_all_environments(false);
}
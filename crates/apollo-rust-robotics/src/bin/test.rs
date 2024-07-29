use apollo_rust_preprocessor::{ResourcesRootDirectoryPreprocessorTrait, ResourcesRootDirectoryTrait};
use apollo_rust_robot_modules::ResourcesRobotsDirectory;

fn main() {
    let r = ResourcesRobotsDirectory::new_default();
    r.preprocess_all(false);
}
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_preprocessor::{ResourcesSubDirectoryTrait};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ur5");

    s.preprocess_robot(false, false);
}
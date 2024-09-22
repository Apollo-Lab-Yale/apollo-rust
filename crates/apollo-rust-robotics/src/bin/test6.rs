use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_preprocessor::ResourcesRootDirectoryTrait;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    r.preprocess_all_robots(false);
}
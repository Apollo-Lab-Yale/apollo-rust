use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{PreprocessForceBuildModulesTrait, ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("h1_test");
    let mut c = s.to_chain_nalgebra();

    c.refine_link_shapes_skips_module();
}
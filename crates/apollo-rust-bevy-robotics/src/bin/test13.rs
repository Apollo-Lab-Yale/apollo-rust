use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{PreprocessForceBuildModulesTrait, ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("xarm7");
    let c = s.to_chain_nalgebra();

    c.refine_link_shapes_skips_module();

}
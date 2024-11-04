use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{PreprocessForceBuildModulesTrait, ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("b1z1_robotiq_140").to_chain_nalgebra();
    c.refine_link_shapes_skips_module();
}
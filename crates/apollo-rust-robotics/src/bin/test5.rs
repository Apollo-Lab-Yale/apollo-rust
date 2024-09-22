use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{ChainBuildersTrait, PreprocessForceBuildModulesTrait, ToChainNalgebra};
use apollo_rust_robotics_core::ChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("b1");
    let mut c = s.to_chain_nalgebra();
    c.refine_link_shapes_skips_module();
}
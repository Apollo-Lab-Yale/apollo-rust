use apollo_rust_linalg::V;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_preprocessor::{ResourcesRootDirectoryTrait, ResourcesSubDirectoryTrait};
use apollo_rust_robotics::{ChainBuildersTrait, PreprocessForceBuildModulesTrait, ToChainNalgebra};
use apollo_rust_robotics_core::ChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_directory();
    let s = r.get_subdirectory("b1");
    let mut c = s.to_chain_nalgebra();
    let state = V::from_column_slice(&[8.04123, 7.17793, 8.97709, 4.39334, 5.37669, 2.27574, 9.92647, 9.73701, 2.19179, 5.31817, 3.15308, 3.56153]);
    let vec = c.fk(&state);
    println!("{:?}", vec);
    // c.refine_link_shapes_skips_module();
}
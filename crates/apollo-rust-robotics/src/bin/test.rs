use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{ApolloDVectorTrait};
use apollo_rust_preprocessor::ResourcesRootDirectoryTrait;
use apollo_rust_preprocessor::robot_modules_preprocessor::{ApolloChainCreator, ChainCreatorAction};
use apollo_rust_robot_modules::ResourcesRootDirectory;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_environments_dir());

    ApolloChainCreator::new("test")
        .add_action(ChainCreatorAction::AddSingleLinkFromGlbFile {
            fp: PathBuf::new_from_desktop_dir().append("untitled.glb"),
            object_name: "tester".to_string(),
            parent_object: None,
            base_offset: Default::default(),
            scale: [1., 1., 1.],
        }).create_and_preprocess(&r, true);
}
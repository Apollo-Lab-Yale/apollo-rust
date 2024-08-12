use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_preprocessor::{ResourcesRootDirectoryTrait, ResourcesSubDirectoryTrait};
use apollo_rust_preprocessor::robot_modules_preprocessor::{ApolloChainCreator, ChainCreatorAction};
use apollo_rust_robot_modules::ResourcesRootDirectory;

fn main() {


    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_environments_dir());
    ApolloChainCreator::new("test")
        .add_action(ChainCreatorAction::AddSingleLinkFromStlFile {
            fp: PathBuf::new_from_desktop_dir().append("untitled.stl"),
            object_name: "tester".to_string(),
            parent_object: None,
            base_offset: Default::default(),
            scale: [1.,1.,1.],
        }).create_and_preprocess(&r, false);

    // r.preprocess_all_environments(true);
    // let s = r.get_subdirectory("test");
    // s.preprocess_environment(false);
    // a.create_and_preprocess(&r, false);
}
use std::path::PathBuf;
use apollo_rust_environment_modules::ResourcesEnvironmentsDirectory;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_preprocessor::{ResourcesRootDirectoryPreprocessorTrait, ResourcesRootDirectoryTrait};
use apollo_rust_preprocessor::environment_modules_preprocessor::{ApolloEnvironmentCreator, EnvironmentCreatorAction};

fn main() {
    /*
    let a = ApolloEnvironmentCreator::new("test2")
        .add_action(EnvironmentCreatorAction::AddAlreadyExistingEnvironment {
            name: "test".to_string(),
            base_offset: Default::default(),
            scale: [1., 1., 1.],
        });
    let r = ResourcesEnvironmentsDirectory::new(PathBuf::new_from_documents_dir().append("apollo-robots-dir/environments"));
    a.create_and_preprocess(&r, false);
    */


    let r = ResourcesEnvironmentsDirectory::new(PathBuf::new_from_documents_dir().append("apollo-robots-dir/environments"));
    r.preprocess_all(false);

    /*
    let c = ApolloEnvironmentCreator::new("woah").add_action(EnvironmentCreatorAction::AddAlreadyExistingEnvironment {
        name: "test".to_string(),
        base_offset: Default::default(),
        scale: [1., 1., 1.],
    });
    */

    // let s = r.get_subdirectory("test");
    // s.preprocess(true);
    // c.create_and_preprocess(&r, true);
}
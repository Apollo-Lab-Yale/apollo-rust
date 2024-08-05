use apollo_rust_preprocessor::environment_modules_preprocessor::{ApolloEnvironmentCreator, EnvironmentCreatorAction};

fn main() {
    let mut a = ApolloEnvironmentCreator::new();
    a.add_action(EnvironmentCreatorAction::AddSceneFromGlbFile {
        fp: Default::default(),
        parent_object: None,
        transform: Default::default(),
        scale: [1.0, 1.0, 1.0],
    });
    println!("{:?}", a);
}
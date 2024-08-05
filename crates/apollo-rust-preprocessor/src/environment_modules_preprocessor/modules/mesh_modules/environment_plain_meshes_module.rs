use apollo_rust_environment_modules::mesh_modules::environment_plain_meshes_module::ApolloEnvironmentPlainMeshesModule;
use apollo_rust_environment_modules::ResourcesSingleEnvironmentDirectory;
use crate::environment_modules_preprocessor::ApolloEnvironmentCreator;

pub trait EnvironmentPlainMeshesModuleBuilders : Sized {
    fn build_from_environment_creator(s: &ResourcesSingleEnvironmentDirectory, environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String>;
}
impl EnvironmentPlainMeshesModuleBuilders for ApolloEnvironmentPlainMeshesModule {
    fn build_from_environment_creator(_s: &ResourcesSingleEnvironmentDirectory, _environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String> {
        todo!()
    }
}
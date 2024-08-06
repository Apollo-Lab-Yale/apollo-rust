use apollo_rust_environment_modules::environment_chain_module::ApolloEnvironmentChainModule;
use apollo_rust_environment_modules::environment_description_module::ApolloEnvironmentDescriptionModule;
use apollo_rust_environment_modules::ResourcesSingleEnvironmentDirectory;
use apollo_rust_robot_modules::chain_module::ApolloChainModule;
use crate::PreprocessorModule;
use crate::robot_modules_preprocessor::modules::chain_module::ChainModuleBuilders;
use crate::utils::progress_bar::ProgressBarWrapper;

impl PreprocessorModule for ApolloEnvironmentChainModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "chain_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let description_module = ApolloEnvironmentDescriptionModule::load_or_build(s, false).expect("error");
        let urdf_module = &description_module.urdf_module;
        let chain_module = ApolloChainModule::build_from_urdf_module(urdf_module, progress_bar).expect("error");
        Ok(Self(chain_module))
    }
}
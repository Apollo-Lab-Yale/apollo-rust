use apollo_rust_environment_modules::environment_bounds_module::ApolloEnvironmentBoundsModule;
use apollo_rust_environment_modules::environment_description_module::ApolloEnvironmentDescriptionModule;
use apollo_rust_environment_modules::environment_dof_module::ApolloEnvironmentDOFModule;
use apollo_rust_environment_modules::ResourcesSingleEnvironmentDirectory;
use apollo_rust_robot_modules::bounds_module::ApolloBoundsModule;
use crate::PreprocessorModule;
use crate::robot_modules_preprocessor::modules::bounds_module::BoundsModuleBuilders;
use crate::utils::progress_bar::ProgressBarWrapper;

impl PreprocessorModule for ApolloEnvironmentBoundsModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "bounds_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let description_module = ApolloEnvironmentDescriptionModule::load_or_build(s, false).expect("error");
        let urdf_module = &description_module.urdf_module;
        let dof_module = ApolloEnvironmentDOFModule::load_or_build(s, false).expect("error");
        let dof_module = &dof_module.0;
        let module = ApolloBoundsModule::build_from_urdf_and_dof_module(urdf_module, dof_module, progress_bar).expect("error");
        Ok(Self(module))
    }
}
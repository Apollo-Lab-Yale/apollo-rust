use apollo_rust_environment_modules::mesh_modules::environment_convex_hull_meshes_module::ApolloEnvironmentConvexHullMeshesModule;
use apollo_rust_environment_modules::ResourcesSingleEnvironmentDirectory;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use crate::PreprocessorModule;
use crate::robot_modules_preprocessor::modules::mesh_modules::convex_hull_meshes_module::ConvexHullMeshesModuleBuilders;
use crate::utils::progress_bar::ProgressBarWrapper;

impl PreprocessorModule for ApolloEnvironmentConvexHullMeshesModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/convex_hull_meshes_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let ss = ResourcesSingleRobotDirectory {
            robot_name: s.environment_name.clone(),
            robots_directory: s.environments_directory.clone(),
            directory: s.directory.clone(),
        };
        let module = ApolloConvexHullMeshesModule::build_from_plain_meshes_module(&ss, progress_bar).expect("error");
        progress_bar.done_preset();
        return Ok(Self(module));
    }
}
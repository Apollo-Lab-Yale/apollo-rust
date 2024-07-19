use apollo_rust_robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::RobotPreprocessorSingleRobotDirectory;
use crate::RobotPreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

impl RobotPreprocessorModule for ApolloBoundsModule {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String {
        "bounds_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf = ApolloURDFModule::load_or_build(s, false).expect("error");
        let dof_module = ApolloDOFModule::load_or_build(s, false).expect("error");

        let mut bounds = vec![];
        let mut dof_lower_bounds = vec![];
        let mut dof_upper_bounds = vec![];

        dof_module.dof_idx_to_joint_idx_mapping.iter().for_each(|joint_idx| {
            let joint = &urdf.joints[*joint_idx];
            let limit = &joint.limit;

            if limit.lower == limit.upper {
                bounds.push( (limit.lower, limit.upper + 0.0000001) );
                dof_lower_bounds.push(limit.lower);
                dof_upper_bounds.push(limit.upper + 0.0000001);
            } else {
                bounds.push( (limit.lower, limit.upper) );
                dof_lower_bounds.push(limit.lower);
                dof_upper_bounds.push(limit.upper);
            }
        });

        progress_bar.done_preset();
        Ok(ApolloBoundsModule {
            bounds,
            dof_lower_bounds,
            dof_upper_bounds,
        })
    }
}
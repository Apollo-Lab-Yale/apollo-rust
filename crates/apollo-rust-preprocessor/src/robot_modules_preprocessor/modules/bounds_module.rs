use crate::utils::progress_bar::ProgressBarWrapper;
use crate::PreprocessorModule;
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_modules::ResourcesSubDirectory;

pub trait BoundsModuleBuilders: Sized {
    fn build_from_urdf_and_dof_module(
        urdf_module: &ApolloURDFModule,
        dof_module: &ApolloDOFModule,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
}
impl BoundsModuleBuilders for ApolloBoundsModule {
    fn build_from_urdf_and_dof_module(
        urdf_module: &ApolloURDFModule,
        dof_module: &ApolloDOFModule,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let mut bounds = vec![];
        let mut dof_lower_bounds = vec![];
        let mut dof_upper_bounds = vec![];

        dof_module
            .dof_idx_to_joint_idx_mapping
            .iter()
            .for_each(|joint_idx| {
                let joint = &urdf_module.joints[*joint_idx];
                let limit = &joint.limit;

                if limit.lower == limit.upper {
                    bounds.push((limit.lower, limit.upper + 0.0000001));
                    dof_lower_bounds.push(limit.lower);
                    dof_upper_bounds.push(limit.upper + 0.0000001);
                } else {
                    bounds.push((limit.lower, limit.upper));
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

use apollo_rust_file::ApolloPathBufTrait;

impl<P: ApolloPathBufTrait + Clone> PreprocessorModule<P> for ApolloBoundsModule {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "bounds_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let urdf = ApolloURDFModule::load_or_build(s, false).expect("error");
        let dof_module = ApolloDOFModule::load_or_build(s, false).expect("error");

        Self::build_from_urdf_and_dof_module(&urdf, &dof_module, progress_bar)
    }
}

use apollo_rust_robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::ResourcesSingleRobotDirectory;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFJointType, ApolloURDFModule};
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait DOFModuleBuilders: Sized {
    fn build_from_urdf_module(urdf_module: &ApolloURDFModule, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;
}
impl DOFModuleBuilders for ApolloDOFModule {
    fn build_from_urdf_module(urdf_module: &ApolloURDFModule, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let mut num_dofs = 0;
        let mut dof_idx_to_joint_mapping_idx = vec![];
        let mut joint_idx_to_dofs_mapping = vec![];

        urdf_module.joints.iter().enumerate().for_each(|(i, j)| {
            match j.joint_type {
                ApolloURDFJointType::Revolute => {
                    dof_idx_to_joint_mapping_idx.push(i);
                    joint_idx_to_dofs_mapping.push(vec![num_dofs]);
                    num_dofs += 1;
                }
                ApolloURDFJointType::Continuous => {
                    dof_idx_to_joint_mapping_idx.push(i);
                    joint_idx_to_dofs_mapping.push(vec![num_dofs]);
                    num_dofs += 1;
                }
                ApolloURDFJointType::Prismatic => {
                    dof_idx_to_joint_mapping_idx.push(i);
                    joint_idx_to_dofs_mapping.push(vec![num_dofs]);
                    num_dofs += 1;
                }
                ApolloURDFJointType::Fixed => {
                    joint_idx_to_dofs_mapping.push(vec![]);
                }
                ApolloURDFJointType::Floating => {
                    for _ in 0..6 { dof_idx_to_joint_mapping_idx.push(i); }
                    joint_idx_to_dofs_mapping.push(vec![num_dofs, num_dofs+1, num_dofs+2, num_dofs+3, num_dofs+4, num_dofs+5]);
                    num_dofs += 6;
                }
                ApolloURDFJointType::Planar => {
                    for _ in 0..3 { dof_idx_to_joint_mapping_idx.push(i); }
                    joint_idx_to_dofs_mapping.push(vec![num_dofs, num_dofs+1, num_dofs+2]);
                    num_dofs += 3;
                }
                ApolloURDFJointType::Spherical => {
                    for _ in 0..3 { dof_idx_to_joint_mapping_idx.push(i); }
                    joint_idx_to_dofs_mapping.push(vec![num_dofs, num_dofs+1, num_dofs+2]);
                    num_dofs += 3;
                }
            }
        });

        progress_bar.done_preset();

        Ok(Self {
            num_dofs,
            dof_idx_to_joint_idx_mapping: dof_idx_to_joint_mapping_idx,
            joint_idx_to_dof_idxs_mapping: joint_idx_to_dofs_mapping
        })
    }
}

impl PreprocessorModule for ApolloDOFModule {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "dof_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false).expect("could not build ApolloDOFModule because ApolloURDFModule could not be built or loaded");

        return Self::build_from_urdf_module(&urdf_module, progress_bar);
    }
}
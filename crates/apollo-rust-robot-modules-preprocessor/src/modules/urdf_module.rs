use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFInertial, ApolloURDFJoint, ApolloURDFJointType, ApolloURDFLink, ApolloURDFLinkName, ApolloURDFMaterial, ApolloURDFModule, ApolloURDFPose};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFJointNalgebra;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_robotics_core::{RobotPreprocessorRobotsDirectory, RobotPreprocessorSingleRobotDirectory};
use apollo_rust_spatial::vectors::V3;
use crate::{AdjustedRobot, AttachmentPoint, CombinedRobot, RobotPreprocessorModule};
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait URDFModuleBuilders: Sized {
    fn build_from_urdf_path(urdf_path: &PathBuf) -> Result<Self, String>;
    fn build_from_adjusted_robot(s: &RobotPreprocessorSingleRobotDirectory, adjusted_robot: &AdjustedRobot) -> Result<Self, String>;
    fn build_from_combined_robot(s: &RobotPreprocessorSingleRobotDirectory, combined_robot: &CombinedRobot) -> Result<Self, String>;
}
impl URDFModuleBuilders for ApolloURDFModule {
    fn build_from_urdf_path(urdf_path: &PathBuf) -> Result<Self, String> {
        let robot = urdf_rs::read_file(urdf_path);
        return match robot {
            Ok(robot) => {
                Ok( Self {
                    name: robot.name.clone(),
                    links: robot.links.iter().map(|x| ApolloURDFLink::from_link(x)).collect(),
                    joints: robot.joints.iter().map(|x| ApolloURDFJoint::from_joint(x)).collect(),
                    materials: robot.materials.iter().map(|x| ApolloURDFMaterial::from_material(x)).collect(),
                } )
            }
            Err(e) => {
                Err(format!("Unable to load urdf.  Error: {:?}", e))
            }
        }
    }

    fn build_from_adjusted_robot(s: &RobotPreprocessorSingleRobotDirectory, adjusted_robot: &AdjustedRobot) -> Result<Self, String> {
        let root = RobotPreprocessorRobotsDirectory::new(s.robots_directory().clone());

        let core_robot_urdf_module = ApolloURDFModule::load_or_build(&root.get_robot_subdirectory(&adjusted_robot.base_robot_name), false).expect("error");

        let name = adjusted_robot.name.clone();
        let links: Vec<ApolloURDFLink> = core_robot_urdf_module.links.iter().filter(|x| !adjusted_robot.deleted_base_links.contains(&x.name)).map(|x| x.clone()).collect();
        let mut joints: Vec<ApolloURDFJoint> = core_robot_urdf_module.joints.iter().filter(|x| !adjusted_robot.deleted_base_joints.contains(&x.name)).map(|x| x.clone()).collect();
        let materials = core_robot_urdf_module.materials.clone();

        adjusted_robot.fixed_base_joints.iter().for_each(|(joint_name, fixed_dofs)| {
            let joint_idx = joints.iter().position(|x| &x.name == joint_name).expect(&format!("joint with name {} does not exist.  Cannot build from adjusted robot", joint_name));
            let joint = &mut joints[joint_idx];
            let joint_nalgebra = ApolloURDFJointNalgebra::from_apollo_urdf_joint(joint);
            let variable_transform = RobotKinematicsFunctions::get_joint_variable_transform(&joint.joint_type, &V3::from_column_slice(&joint.axis.xyz), fixed_dofs);
            let new_origin = joint_nalgebra.origin.ise3q.group_operator(&variable_transform);
            let xyz = new_origin.0.translation.vector.xyz();
            let rpy = new_origin.0.rotation.euler_angles();
            let new_origin_as_pose = ApolloURDFPose::new([xyz[0], xyz[1], xyz[2]], [rpy.0, rpy.1, rpy.2]);
            joint.origin = new_origin_as_pose;
            joint.joint_type = ApolloURDFJointType::Fixed;
        });

        Ok(Self {
            name,
            links,
            joints,
            materials,
        })
    }

    fn build_from_combined_robot(s: &RobotPreprocessorSingleRobotDirectory, combined_robot: &CombinedRobot) -> Result<Self, String> {
        let root = RobotPreprocessorRobotsDirectory::new(s.robots_directory().clone());

        let name = combined_robot.name.clone();
        let mut links = vec![];
        let mut joints = vec![];
        let mut materials = vec![];

        links.push(ApolloURDFLink {
            name: "combined_robot_world".to_string(),
            inertial: ApolloURDFInertial::default(),
            visual: vec![],
            collision: vec![],
        });

        combined_robot.attached_robots.iter().enumerate().for_each(|(i, x)| {
            let ss = root.get_robot_subdirectory(&x.robot_name);
            let attached_robot = x;
            let urdf_module = ApolloURDFModule::load_or_build(&ss, false).expect("error");
            let chain_module = ApolloChainModule::load_or_build(&ss, false).expect("error");

            let mut new_joint = ApolloURDFJoint::default();
            new_joint.name = format!("connection_joint_robot_{}", i);
            new_joint.parent = match &attached_robot.attachment_point {
                AttachmentPoint::World => { ApolloURDFLinkName { link: "combined_robot_world".to_string() } }
                AttachmentPoint::Link { robot_idx, link_name } => { ApolloURDFLinkName { link: format!("robot_{}_{}", *robot_idx, link_name) } }
            };
            new_joint.child = ApolloURDFLinkName { link: format!("robot_{}_{}", i, urdf_module.links[chain_module.root_idx].name.clone()) };
            new_joint.joint_type = attached_robot.joint_type.clone();
            new_joint.origin = attached_robot.origin.clone();
            new_joint.safety_controller = attached_robot.safety_controller.clone();
            new_joint.dynamics = attached_robot.dynamics.clone();
            new_joint.limit = attached_robot.limit.clone();
            new_joint.mimic = None;

            joints.push(new_joint);

            urdf_module.links.iter().for_each(|x| {
                let mut link_clone = x.clone();
                link_clone.name = format!("robot_{}_{}", i, link_clone.name);
                links.push(link_clone);
            });

            urdf_module.joints.iter().for_each(|x| {
                let mut joint_clone = x.clone();

                joint_clone.name = format!("robot_{}_{}", i, joint_clone.name);
                joint_clone.parent = ApolloURDFLinkName { link: format!("robot_{}_{}", i, joint_clone.parent.link) };
                joint_clone.child = ApolloURDFLinkName { link: format!("robot_{}_{}", i, joint_clone.child.link) };
                joints.push(joint_clone);
            });

            materials.extend(urdf_module.materials);
        });

        Ok(Self {
            name,
            links,
            joints,
            materials,
        })
    }
}

impl RobotPreprocessorModule for ApolloURDFModule {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String {
        "urdf_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let fp = s.directory().clone();
        let files = fp.get_all_items_in_directory(false, false, true, false);
        for file in files {
            if file.extension().expect("error").to_str().expect("error") == "urdf" {
                progress_bar.done_preset();
                let fp = file.clone();
                progress_bar.done_preset();
                return Self::build_from_urdf_path(&fp);
            }
        }

        let fp = s.directory().clone();
        let fp = fp.append("combined_robot_module/module.json");
        let combined_robot = fp.load_object_from_json_file_result::<CombinedRobot>();
        if let Ok(combined_robot) = combined_robot {
            progress_bar.done_preset();
            return Self::build_from_combined_robot(s, &combined_robot);
        }

        let fp = s.directory().clone();
        let fp = fp.append("adjusted_robot_module/module.json");
        let adjusted_robot = fp.load_object_from_json_file_result::<AdjustedRobot>();
        if let Ok(adjusted_robot) = adjusted_robot {
            progress_bar.done_preset();
            return Self::build_from_adjusted_robot(s, &adjusted_robot);
        }

        return Err(format!("urdf module could not be constructed in directory {:?}", s.directory));
    }
}

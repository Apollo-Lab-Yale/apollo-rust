use crate::robot_modules_preprocessor::{
    AdjustedRobot, ApolloChainCreator, AttachmentPoint, ChainCreatorAction, CombinedRobot,
};
use crate::utils::progress_bar::ProgressBarWrapper;
use crate::PreprocessorModule;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::ToMeshObjectScene;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::urdf_module::{
    ApolloURDFInertial, ApolloURDFJoint, ApolloURDFJointType, ApolloURDFLink, ApolloURDFLinkName,
    ApolloURDFMaterial, ApolloURDFModule, ApolloURDFPose,
};
use apollo_rust_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFJointNalgebra;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use std::path::PathBuf;

pub trait URDFModuleBuilders<P = PathBuf>: Sized
where
    P: ApolloPathBufTrait + Clone,
{
    fn build_from_urdf_path(urdf_path: &P) -> Result<Self, String>;
    fn build_from_adjusted_robot(
        s: &ResourcesSubDirectory<P>,
        adjusted_robot: &AdjustedRobot,
    ) -> Result<Self, String>;
    fn build_from_combined_robot(
        s: &ResourcesSubDirectory<P>,
        combined_robot: &CombinedRobot,
    ) -> Result<Self, String>;
    fn build_from_environment_creator(
        s: &ResourcesSubDirectory<P>,
        environment_creator: &ApolloChainCreator,
    ) -> Result<Self, String>;
}
impl<P: ApolloPathBufTrait + Clone> URDFModuleBuilders<P> for ApolloURDFModule {
    fn build_from_urdf_path(urdf_path: &P) -> Result<Self, String> {
        let content = urdf_path.read_file_contents_to_string();
        let robot = urdf_rs::read_from_string(&content);
        return match robot {
            Ok(robot) => Ok(Self {
                name: robot.name.clone(),
                links: robot
                    .links
                    .iter()
                    .map(|x| ApolloURDFLink::from_link(x))
                    .collect(),
                joints: robot
                    .joints
                    .iter()
                    .map(|x| ApolloURDFJoint::from_joint(x))
                    .collect(),
                materials: robot
                    .materials
                    .iter()
                    .map(|x| ApolloURDFMaterial::from_material(x))
                    .collect(),
            }),
            Err(e) => Err(format!("Unable to load urdf.  Error: {:?}", e)),
        };
    }

    fn build_from_adjusted_robot(
        s: &ResourcesSubDirectory<P>,
        adjusted_robot: &AdjustedRobot,
    ) -> Result<Self, String> {
        let root = ResourcesRootDirectory::new(s.root_directory.clone(), s.resources_type);

        let core_robot_urdf_module = ApolloURDFModule::load_or_build(
            &root.get_subdirectory(&adjusted_robot.base_robot_name),
            false,
        )
        .expect("error");
        // ... rest stays same but s is generic ...

        let name = adjusted_robot.name.clone();
        let links: Vec<ApolloURDFLink> = core_robot_urdf_module
            .links
            .iter()
            .filter(|x| !adjusted_robot.deleted_base_links.contains(&x.name))
            .map(|x| x.clone())
            .collect();
        let mut joints: Vec<ApolloURDFJoint> = core_robot_urdf_module
            .joints
            .iter()
            .filter(|x| !adjusted_robot.deleted_base_joints.contains(&x.name))
            .map(|x| x.clone())
            .collect();
        let materials = core_robot_urdf_module.materials.clone();

        adjusted_robot
            .fixed_base_joints
            .iter()
            .for_each(|(joint_name, fixed_dofs)| {
                let joint_idx = joints
                    .iter()
                    .position(|x| &x.name == joint_name)
                    .expect(&format!(
                        "joint with name {} does not exist.  Cannot build from adjusted robot",
                        joint_name
                    ));
                let joint = &mut joints[joint_idx];
                let joint_nalgebra = ApolloURDFJointNalgebra::from_apollo_urdf_joint(joint);
                let variable_transform = RobotKinematicsFunctions::get_joint_variable_transform(
                    &joint.joint_type,
                    &V3::from_column_slice(&joint.axis.xyz),
                    fixed_dofs,
                );
                let new_origin = joint_nalgebra
                    .origin
                    .ise3q
                    .group_operator(&variable_transform);
                let xyz = new_origin.0.translation.vector.xyz();
                let rpy = new_origin.0.rotation.euler_angles();
                let new_origin_as_pose =
                    ApolloURDFPose::new([xyz[0], xyz[1], xyz[2]], [rpy.0, rpy.1, rpy.2]);
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

    fn build_from_combined_robot(
        s: &ResourcesSubDirectory<P>,
        combined_robot: &CombinedRobot,
    ) -> Result<Self, String> {
        let root = ResourcesRootDirectory::new(s.root_directory.clone(), s.resources_type);

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

        combined_robot
            .attached_robots
            .iter()
            .enumerate()
            .for_each(|(i, x)| {
                let ss = root.get_subdirectory(&x.robot_name);
                let attached_robot = x;
                let urdf_module = ApolloURDFModule::load_or_build(&ss, false).expect("error");
                let chain_module = ApolloChainModule::load_or_build(&ss, false).expect("error");

                let mut new_joint = ApolloURDFJoint::default();
                new_joint.name = format!("connection_joint_robot_{}", i);
                new_joint.parent = match &attached_robot.attachment_point {
                    AttachmentPoint::World => ApolloURDFLinkName {
                        link: "combined_robot_world".to_string(),
                    },
                    AttachmentPoint::Link {
                        robot_idx,
                        link_name,
                    } => ApolloURDFLinkName {
                        link: format!("robot_{}_{}", *robot_idx, link_name),
                    },
                };
                new_joint.child = ApolloURDFLinkName {
                    link: format!(
                        "robot_{}_{}",
                        i,
                        urdf_module.links[chain_module.root_idx].name.clone()
                    ),
                };
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
                    joint_clone.parent = ApolloURDFLinkName {
                        link: format!("robot_{}_{}", i, joint_clone.parent.link),
                    };
                    joint_clone.child = ApolloURDFLinkName {
                        link: format!("robot_{}_{}", i, joint_clone.child.link),
                    };
                    match &mut joint_clone.mimic {
                        None => {}
                        Some(mimic) => {
                            mimic.joint = format!("robot_{}_{}", i, mimic.joint);
                        }
                    }
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

    fn build_from_environment_creator(
        s: &ResourcesSubDirectory<P>,
        environment_creator: &ApolloChainCreator,
    ) -> Result<Self, String> {
        let environment_name = s.name.clone();
        let mut links = vec![];
        let mut joints = vec![];

        links.push(ApolloURDFLink {
            name: "world_environment_origin".to_string(),
            inertial: Default::default(),
            visual: vec![],
            collision: vec![],
        });
        // link_scales.push([1.,1.,1.]);
        // link_simulation_modes.push(EnvironmentLinkSimulationMode::Passive);

        environment_creator.actions.iter().for_each(|action| {
            match action {
                ChainCreatorAction::AddAlreadyExistingChain {
                    name, base_offset, ..
                } => {
                    let environments_directory =
                        ResourcesRootDirectory::new(s.root_directory.clone(), s.resources_type);
                    let ss = environments_directory.get_subdirectory(name);
                    let urdf_module = ApolloURDFModule::load_or_build(&ss, false).expect("error");

                    let mut joints_clone = urdf_module.joints.clone();
                    joints_clone.iter_mut().for_each(|joint| {
                        if joint.parent.link == "world_environment_origin" {
                            let p = ISE3q::new(I3::from_slices_euler_angles(
                                &joint.origin.xyz,
                                &joint.origin.rpy,
                            ));
                            let res = p.group_operator(base_offset);
                            let xyz = res.0.translation.vector.xyz().as_slice().to_vec();
                            let rpy = res.0.rotation.euler_angles();
                            joint.origin.xyz = [xyz[0], xyz[1], xyz[2]];
                            joint.origin.rpy = [rpy.0, rpy.1, rpy.2];
                        }
                    });
                    // let scales_clone: Vec<[f64; 3]> = description_module.link_scales.iter().map(|x| [x[0] * scale[0], x[1] * scale[1], x[2] * scale[2]]).collect();

                    urdf_module.links.iter().enumerate().for_each(|(_i, x)| {
                        if x.name != "world_environment_origin" {
                            links.push(x.clone());
                            // link_scales.push(scales_clone[i]);
                            // link_simulation_modes.push(description_module.link_simulation_modes[i].clone());
                        }
                    });
                    joints.extend(joints_clone);
                }
                ChainCreatorAction::AddSingleLinkFromStlFile {
                    object_name,
                    parent_object,
                    base_offset,
                    ..
                } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(
                        object_name,
                        parent_object,
                        base_offset,
                    );
                    links.push(link);
                    joints.push(joint);
                    // link_scales.push(scale.clone());
                    // link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                ChainCreatorAction::AddSingleLinkFromObjFile {
                    object_name,
                    parent_object,
                    base_offset,
                    ..
                } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(
                        object_name,
                        parent_object,
                        base_offset,
                    );
                    links.push(link);
                    joints.push(joint);
                    // link_scales.push(scale.clone());
                    // link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                ChainCreatorAction::AddSingleLinkFromGlbFile {
                    object_name,
                    parent_object,
                    base_offset,
                    ..
                } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(
                        object_name,
                        parent_object,
                        base_offset,
                    );
                    links.push(link);
                    joints.push(joint);
                    // link_scales.push(scale.clone());
                    // link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                ChainCreatorAction::AddSceneFromGlbFile {
                    scene_name,
                    fp,
                    parent_object,
                    transform,
                    ..
                } => {
                    fp.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"])
                        .expect("error");

                    let target = s
                        .directory
                        .clone()
                        .append("mesh_modules/glb_scenes")
                        .append(scene_name)
                        .append("scene.glb");
                    let mesh_object_scene = if target.path_exists() {
                        load_gltf_file(&target.to_path_buf())
                            .expect("error")
                            .to_mesh_object_scene()
                    } else {
                        assert!(fp.path_exists());
                        let fp_p = P::new_from_path(&fp);
                        fp_p.copy_file_to_destination_file_path(&target);
                        load_gltf_file(&fp_p).expect("error").to_mesh_object_scene()
                    };

                    mesh_object_scene.nodes.iter().for_each(|node| {
                        let parent_link_name = match &node.parent_node {
                            None => match parent_object {
                                None => "world_environment_origin".to_string(),
                                Some(p) => p.clone(),
                            },
                            Some(s) => s.clone(),
                        };

                        let link = ApolloURDFLink {
                            name: node.name.clone(),
                            inertial: Default::default(),
                            visual: vec![],
                            collision: vec![],
                        };

                        let joint_name =
                            format!("joint_between_{}_and_{}", parent_link_name, node.name);

                        let mut offset_from_parent =
                            node.local_space_mesh_object.offset_from_parent.clone();
                        if parent_object.is_none() {
                            offset_from_parent = offset_from_parent * transform.0;
                        }

                        let xyz = offset_from_parent
                            .translation
                            .vector
                            .xyz()
                            .as_slice()
                            .to_vec();
                        let rpy = offset_from_parent.rotation.euler_angles();

                        let joint = ApolloURDFJoint {
                            name: joint_name,
                            joint_type: ApolloURDFJointType::Floating,
                            origin: ApolloURDFPose {
                                xyz: [xyz[0], xyz[1], xyz[2]],
                                rpy: [rpy.0, rpy.1, rpy.2],
                            },
                            parent: ApolloURDFLinkName {
                                link: parent_link_name.clone(),
                            },
                            child: ApolloURDFLinkName {
                                link: node.name.clone(),
                            },
                            axis: Default::default(),
                            limit: Default::default(),
                            dynamics: None,
                            mimic: None,
                            safety_controller: None,
                        };

                        links.push(link);
                        joints.push(joint);
                        // link_scales.push(scale.clone());
                        // link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                    });
                }
                _ => {}
            }
        });

        // assert_eq!(links.len(), link_scales.len());
        // assert_eq!(links.len(), link_simulation_modes.len());

        environment_creator
            .actions
            .iter()
            .for_each(|action| match action {
                ChainCreatorAction::SetJointType {
                    parent_object,
                    child_object,
                    joint_type,
                } => {
                    let idx = get_joint_idx_from_parent_and_child_names(
                        parent_object,
                        child_object,
                        &joints,
                    )
                    .expect("error");
                    joints[idx].joint_type = joint_type.clone();
                }
                ChainCreatorAction::SetJointAxis {
                    parent_object,
                    child_object,
                    axis,
                } => {
                    let idx = get_joint_idx_from_parent_and_child_names(
                        parent_object,
                        child_object,
                        &joints,
                    )
                    .expect("error");
                    joints[idx].axis = axis.clone();
                }
                ChainCreatorAction::SetJointLimit {
                    parent_object,
                    child_object,
                    joint_limit,
                } => {
                    let idx = get_joint_idx_from_parent_and_child_names(
                        parent_object,
                        child_object,
                        &joints,
                    )
                    .expect("error");
                    joints[idx].limit = joint_limit.clone();
                }
                ChainCreatorAction::SetObjectMass { object_name, mass } => {
                    let idx = links
                        .iter()
                        .position(|x| &x.name == object_name)
                        .expect("error");
                    links[idx].inertial.mass.value = *mass;
                }
                ChainCreatorAction::SetObjectInertialOrigin { object_name, pose } => {
                    let idx = links
                        .iter()
                        .position(|x| &x.name == object_name)
                        .expect("error");
                    let xyz = pose.0.translation.vector.xyz().as_slice().to_vec();
                    let rpy = pose.0.rotation.euler_angles();
                    links[idx].inertial.origin.xyz = [xyz[0], xyz[1], xyz[2]];
                    links[idx].inertial.origin.rpy = [rpy.0, rpy.1, rpy.2];
                }
                ChainCreatorAction::SetObjectInertia {
                    object_name,
                    ixx,
                    ixy,
                    ixz,
                    iyy,
                    iyz,
                    izz,
                } => {
                    let idx = links
                        .iter()
                        .position(|x| &x.name == object_name)
                        .expect("error");
                    links[idx].inertial.inertia.ixx = *ixx;
                    links[idx].inertial.inertia.ixy = *ixy;
                    links[idx].inertial.inertia.ixz = *ixz;
                    links[idx].inertial.inertia.iyy = *iyy;
                    links[idx].inertial.inertia.iyz = *iyz;
                    links[idx].inertial.inertia.izz = *izz;
                }
                _ => {}
            });

        Ok(ApolloURDFModule {
            name: environment_name.clone(),
            links,
            joints,
            materials: vec![],
        })
    }
}

impl<P: ApolloPathBufTrait + Clone> PreprocessorModule<P> for ApolloURDFModule {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "urdf_module".to_string()
    }

    fn current_version() -> String {
        "0.0.3".to_string()
    }

    fn build_raw(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let fp = s.directory.clone();
        let files = fp.get_all_items_in_directory(false, false, true, false);
        for file in files {
            if file.path_exists() && file.path_extension().map(|x| x == "urdf").unwrap_or(false) {
                progress_bar.done_preset();
                let fp = file.clone();
                progress_bar.done_preset();
                return Self::build_from_urdf_path(&fp);
            }
        }

        let fp = s.directory.clone();
        let fp = fp.append("creator_module/module.json");
        let creator_module = fp.load_object_from_json_file_result::<ApolloChainCreator>();
        if let Ok(creator_module) = creator_module {
            progress_bar.done_preset();
            return Self::build_from_environment_creator(s, &creator_module);
        }

        let fp = s.directory.clone();
        let fp = fp.append("combined_robot_module/module.json");
        let combined_robot = fp.load_object_from_json_file_result::<CombinedRobot>();
        if let Ok(combined_robot) = combined_robot {
            progress_bar.done_preset();
            return Self::build_from_combined_robot(s, &combined_robot);
        }

        let fp = s.directory.clone();
        let fp = fp.append("adjusted_robot_module/module.json");
        let adjusted_robot = fp.load_object_from_json_file_result::<AdjustedRobot>();
        if let Ok(adjusted_robot) = adjusted_robot {
            progress_bar.done_preset();
            return Self::build_from_adjusted_robot(s, &adjusted_robot);
        }

        return Err(format!(
            "urdf module could not be constructed in directory {:?}",
            s.directory
        ));
    }
}

fn get_link_and_joint_from_single_mesh_file(
    object_name: &String,
    parent_object: &Option<String>,
    base_offset: &ISE3q,
) -> (ApolloURDFLink, ApolloURDFJoint) {
    let urdf_link = ApolloURDFLink {
        name: object_name.clone(),
        inertial: Default::default(),
        visual: vec![],
        collision: vec![],
    };

    let parent_link_name = match parent_object {
        None => "world_environment_origin".to_string(),
        Some(s) => s.clone(),
    };
    let joint_name = format!("joint_between_{}_and_{}", parent_link_name, object_name);
    let xyz = base_offset.0.translation.vector.xyz().as_slice().to_vec();
    let rpy = base_offset.0.rotation.euler_angles();

    let urdf_joint = ApolloURDFJoint {
        name: joint_name,
        joint_type: ApolloURDFJointType::Floating,
        origin: ApolloURDFPose {
            xyz: [xyz[0], xyz[1], xyz[2]],
            rpy: [rpy.0, rpy.1, rpy.2],
        },
        parent: ApolloURDFLinkName {
            link: parent_link_name.clone(),
        },
        child: ApolloURDFLinkName {
            link: object_name.clone(),
        },
        axis: Default::default(),
        limit: Default::default(),
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };

    return (urdf_link, urdf_joint);
}

fn get_joint_idx_from_parent_and_child_names(
    parent_name: &Option<String>,
    child_name: &String,
    joints: &Vec<ApolloURDFJoint>,
) -> Option<usize> {
    let parent_name = match parent_name {
        None => "world_environment_origin".to_string(),
        Some(s) => s.clone(),
    };
    let joint_name = format!("joint_between_{}_and_{}", parent_name, child_name);
    joints.iter().position(|x| x.name == joint_name)
}

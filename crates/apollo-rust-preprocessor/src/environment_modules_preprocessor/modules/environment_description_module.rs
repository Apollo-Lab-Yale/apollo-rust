use apollo_rust_environment_modules::environment_description_module::{ApolloEnvironmentDescriptionModule, EnvironmentLinkSimulationMode};
use apollo_rust_environment_modules::{ResourcesEnvironmentsDirectory, ResourcesSingleEnvironmentDirectory};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::{ToMeshObjectScene};
use apollo_rust_robot_modules::urdf_module::{ApolloURDFJoint, ApolloURDFJointType, ApolloURDFLink, ApolloURDFLinkName, ApolloURDFModule, ApolloURDFPose};
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::environment_modules_preprocessor::{ApolloEnvironmentCreator, EnvironmentCreatorAction};
use crate::{PreprocessorModule, ResourcesRootDirectoryTrait};
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait EnvironmentDescriptionModuleBuilders : Sized {
    fn build_from_environment_creator(s: &ResourcesSingleEnvironmentDirectory, environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String>;
}
impl EnvironmentDescriptionModuleBuilders for ApolloEnvironmentDescriptionModule {
    fn build_from_environment_creator(s: &ResourcesSingleEnvironmentDirectory, environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String> {

        let environment_name = s.environment_name.clone();
        let mut links = vec![];
        let mut joints = vec![];
        let mut link_scales = vec![];
        let mut link_simulation_modes = vec![];

        links.push(ApolloURDFLink {
            name: "world_environment_origin".to_string(),
            inertial: Default::default(),
            visual: vec![],
            collision: vec![],
        });
        link_scales.push([1.,1.,1.]);
        link_simulation_modes.push(EnvironmentLinkSimulationMode::Passive);
        
        environment_creator.actions.iter().for_each(|action| {
            match action {
                EnvironmentCreatorAction::AddAlreadyExistingEnvironment { name, base_offset, scale, } => {
                    let environments_directory = ResourcesEnvironmentsDirectory::new(s.environments_directory.clone());
                    let ss = environments_directory.get_subdirectory(name);
                    let description_module = ApolloEnvironmentDescriptionModule::load_or_build(&ss, false).expect("error");

                    let mut joints_clone = description_module.urdf_module.joints.clone();
                    joints_clone.iter_mut().for_each(|joint| {
                        if joint.parent.link == "world_environment_origin" {
                            let p = ISE3q::new(I3::from_slices_euler_angles(&joint.origin.xyz, &joint.origin.rpy));
                            let res = p.group_operator(base_offset);
                            let xyz = res.0.translation.vector.xyz().as_slice().to_vec();
                            let rpy = res.0.rotation.euler_angles();
                            joint.origin.xyz = [ xyz[0], xyz[1], xyz[2] ];
                            joint.origin.rpy = [ rpy.0, rpy.1, rpy.2 ];
                        }
                    });
                    let scales_clone: Vec<[f64; 3]> = description_module.link_scales.iter().map(|x| [x[0] * scale[0], x[1] * scale[1], x[2] * scale[2]]).collect();

                    description_module.urdf_module.links.iter().enumerate().for_each(|(i, x)| {
                        if x.name != "world_environment_origin" {
                            links.push(x.clone());
                            link_scales.push(scales_clone[i]);
                            link_simulation_modes.push(description_module.link_simulation_modes[i].clone());
                        }
                    });
                    joints.extend(joints_clone);

                }
                EnvironmentCreatorAction::AddSingleObjectFromStlFile { object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                    link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                EnvironmentCreatorAction::AddSingleObjectFromObjFile {  object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                    link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                EnvironmentCreatorAction::AddSingleObjectFromGlbFile { object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                    link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                }
                EnvironmentCreatorAction::AddSceneFromGlbFile { scene_name, fp, parent_object, transform, scale } => {
                    fp.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"]).expect("error");

                    // let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                    let target = s.directory.clone().append("mesh_modules/glb_scenes").append(scene_name).append("scene.glb");
                    let mesh_object_scene = if target.exists() {
                        load_gltf_file(&target).expect("error").to_mesh_object_scene()
                    } else {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&target);
                        load_gltf_file(fp).expect("error").to_mesh_object_scene()
                    };

                    mesh_object_scene.nodes.iter().for_each(|node| {
                        let parent_link_name = match &node.parent_node {
                            None => {
                                match parent_object {
                                    None => { "world_environment_origin".to_string()  }
                                    Some(p) => { p.clone() }
                                }
                            }
                            Some(s) => { s.clone() }
                        };

                        let link = ApolloURDFLink {
                            name: node.name.clone(),
                            inertial: Default::default(),
                            visual: vec![],
                            collision: vec![],
                        };

                        let joint_name = format!("joint_between_{}_and_{}", parent_link_name, node.name);

                        let mut offset_from_parent = node.local_space_mesh_object.offset_from_parent.clone();
                        if parent_object.is_none() {
                            offset_from_parent = offset_from_parent * transform.0;
                        }

                        let xyz = offset_from_parent.translation.vector.xyz().as_slice().to_vec();
                        let rpy = offset_from_parent.rotation.euler_angles();

                        let joint = ApolloURDFJoint {
                            name: joint_name,
                            joint_type: ApolloURDFJointType::Floating,
                            origin: ApolloURDFPose {
                                xyz: [ xyz[0], xyz[1], xyz[2] ],
                                rpy: [ rpy.0, rpy.1, rpy.2 ],
                            },
                            parent: ApolloURDFLinkName { link: parent_link_name.clone() },
                            child: ApolloURDFLinkName { link: node.name.clone() },
                            axis: Default::default(),
                            limit: Default::default(),
                            dynamics: None,
                            mimic: None,
                            safety_controller: None,
                        };

                        links.push(link);
                        joints.push(joint);
                        link_scales.push(scale.clone());
                        link_simulation_modes.push(EnvironmentLinkSimulationMode::Active);
                    });
                }
                _ => { }
            }
        });

        assert_eq!(links.len(), link_scales.len());
        assert_eq!(links.len(), link_simulation_modes.len());

        environment_creator.actions.iter().for_each(|action| {
            match action {
                EnvironmentCreatorAction::SetJointType { parent_object, child_object, joint_type } => {
                    let idx = get_joint_idx_from_parent_and_child_names(parent_object, child_object, &joints).expect("error");
                    joints[idx].joint_type = joint_type.clone();
                }
                EnvironmentCreatorAction::SetJointAxis { parent_object, child_object, axis } => {
                    let idx = get_joint_idx_from_parent_and_child_names(parent_object, child_object, &joints).expect("error");
                    joints[idx].axis = axis.clone();
                }
                EnvironmentCreatorAction::SetJointLimit { parent_object, child_object, joint_limit } => {
                    let idx = get_joint_idx_from_parent_and_child_names(parent_object, child_object, &joints).expect("error");
                    joints[idx].limit = joint_limit.clone();
                }
                EnvironmentCreatorAction::SetObjectSimulationMode { object_name, mode } => {
                    let idx = links.iter().position(|x| &x.name == object_name).expect("error");
                    link_simulation_modes[idx] = mode.clone();
                }
                EnvironmentCreatorAction::SetObjectMass { object_name, mass } => {
                    let idx = links.iter().position(|x| &x.name == object_name).expect("error");
                    links[idx].inertial.mass.value = *mass;
                }
                EnvironmentCreatorAction::SetObjectInertialOrigin { object_name, pose } => {
                    let idx = links.iter().position(|x| &x.name == object_name).expect("error");
                    let xyz = pose.0.translation.vector.xyz().as_slice().to_vec();
                    let rpy = pose.0.rotation.euler_angles();
                    links[idx].inertial.origin.xyz = [xyz[0], xyz[1], xyz[2]];
                    links[idx].inertial.origin.rpy = [ rpy.0, rpy.1, rpy.2 ];
                }
                EnvironmentCreatorAction::SetObjectInertia { object_name, ixx, ixy, ixz, iyy, iyz, izz } => {
                    let idx = links.iter().position(|x| &x.name == object_name).expect("error");
                    links[idx].inertial.inertia.ixx = *ixx;
                    links[idx].inertial.inertia.ixy = *ixy;
                    links[idx].inertial.inertia.ixz = *ixz;
                    links[idx].inertial.inertia.iyy = *iyy;
                    links[idx].inertial.inertia.iyz = *iyz;
                    links[idx].inertial.inertia.izz = *izz;
                }
                _ => { }
            }
        });

        Ok(Self {
            urdf_module: ApolloURDFModule {
                name: environment_name.clone(),
                links,
                joints,
                materials: vec![],
            },
            link_scales,
            link_simulation_modes
        })
    }
}

fn get_link_and_joint_from_single_mesh_file(object_name: &String, parent_object: &Option<String>, base_offset: &ISE3q) -> (ApolloURDFLink, ApolloURDFJoint) {
    let urdf_link = ApolloURDFLink {
        name: object_name.clone(),
        inertial: Default::default(),
        visual: vec![],
        collision: vec![],
    };

    let parent_link_name = match parent_object {
        None => { "world_environment_origin".to_string() }
        Some(s) => { s.clone() }
    };
    let joint_name = format!("joint_between_{}_and_{}", parent_link_name, object_name);
    let xyz = base_offset.0.translation.vector.xyz().as_slice().to_vec();
    let rpy = base_offset.0.rotation.euler_angles();

    let urdf_joint = ApolloURDFJoint {
        name: joint_name,
        joint_type: ApolloURDFJointType::Floating,
        origin: ApolloURDFPose {
            xyz: [ xyz[0], xyz[1], xyz[2] ],
            rpy: [ rpy.0, rpy.1, rpy.2 ],
        },
        parent: ApolloURDFLinkName { link: parent_link_name.clone() },
        child: ApolloURDFLinkName { link: object_name.clone() },
        axis: Default::default(),
        limit: Default::default(),
        dynamics: None,
        mimic: None,
        safety_controller: None,
    };

    return (urdf_link, urdf_joint);
}

fn get_joint_idx_from_parent_and_child_names(parent_name: &Option<String>, child_name: &String, joints: &Vec<ApolloURDFJoint>) -> Option<usize> {
    let parent_name = match parent_name {
        None => { "world_environment_origin".to_string() }
        Some(s) => { s.clone() }
    };
    let joint_name = format!("joint_between_{}_and_{}", parent_name, child_name);
    joints.iter().position(|x| x.name == joint_name)
}

impl PreprocessorModule for ApolloEnvironmentDescriptionModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "environment_description_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let environment_creator_module = s.directory.clone().append("creator_module").append("module.json");
        let creator = environment_creator_module.load_object_from_json_file_result::<ApolloEnvironmentCreator>().expect("environment must have creator module");

        progress_bar.done_preset();
        return Self::build_from_environment_creator(s, &creator);
    }
}


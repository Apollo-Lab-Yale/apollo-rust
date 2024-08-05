use std::path::PathBuf;
use apollo_rust_environment_modules::environment_description_module::ApolloEnvironmentDescriptionModule;
use apollo_rust_environment_modules::{ResourcesEnvironmentsDirectory, ResourcesSingleEnvironmentDirectory};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::{MeshObjectScene, ToMeshObjectScene};
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

        links.push(ApolloURDFLink {
            name: "world_environment_origin".to_string(),
            inertial: Default::default(),
            visual: vec![],
            collision: vec![],
        });
        
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

                    description_module.urdf_module.links.iter().for_each(|x| { if x.name != "world_environment_origin" { links.push(x.clone()); } });
                    joints.extend(joints_clone);
                    link_scales.extend(scales_clone);
                }
                EnvironmentCreatorAction::AddSingleObjectFromStlFile { object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                }
                EnvironmentCreatorAction::AddSingleObjectFromObjFile {  object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                }
                EnvironmentCreatorAction::AddSingleObjectFromGlbFile { object_name, parent_object, base_offset, scale, .. } => {
                    let (link, joint) = get_link_and_joint_from_single_mesh_file(object_name, parent_object, base_offset);
                    links.push(link);
                    joints.push(joint);
                    link_scales.push(scale.clone());
                }
                EnvironmentCreatorAction::AddSceneFromGlbFile { fp, parent_object, transform, scale } => {
                    fp.verify_extension(&vec!["glb", "GLB", "gltf", "GLTF"]).expect("error");

                    let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                    let target = s.directory.clone().append("mesh_modules/glb_scenes").append(&filename);
                    let mesh_object_scene = if target.exists() {
                        load_gltf_file(&target).expect("error").to_mesh_object_scene()
                    } else {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&target);
                        load_gltf_file(fp).expect("error").to_mesh_object_scene()
                    };

                    mesh_object_scene.nodes.iter().for_each(|node| {
                        let parent_link_name = match &node.parent_node {
                            None => { "world_environment_origin".to_string() }
                            Some(s) => { s.clone() }
                        };

                        let link = ApolloURDFLink {
                            name: node.name.clone(),
                            inertial: Default::default(),
                            visual: vec![],
                            collision: vec![],
                        };

                        let joint_name = format!("joint_between_{}_and_{}", parent_link_name, node.name);
                        let xyz = node.local_space_mesh_object.offset_from_parent.translation.vector.xyz().as_slice().to_vec();
                        let rpy = node.local_space_mesh_object.offset_from_parent.rotation.euler_angles();

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
                    });
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
            link_scales: vec![],
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

impl PreprocessorModule for ApolloEnvironmentDescriptionModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "environment_description_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &Self::SubDirectoryType, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let environment_creator_module = PathBuf::new().append_path(&s.directory).append("creator_module").append("module.json");
        let creator = environment_creator_module.load_object_from_json_file_result::<ApolloEnvironmentCreator>().expect("environment must have creator module");

        progress_bar.done_preset();
        return Self::build_from_environment_creator(s, &creator);
    }
}


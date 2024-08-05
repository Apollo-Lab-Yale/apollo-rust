use apollo_rust_environment_modules::mesh_modules::environment_original_meshes_module::ApolloEnvironmentOriginalMeshesModule;
use apollo_rust_environment_modules::{ResourcesEnvironmentsDirectory, ResourcesSingleEnvironmentDirectory};
use apollo_rust_environment_modules::environment_description_module::ApolloEnvironmentDescriptionModule;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::ToMeshObjectScene;
use apollo_rust_robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use crate::environment_modules_preprocessor::{ApolloEnvironmentCreator, EnvironmentCreatorAction};
use crate::{PreprocessorModule, ResourcesRootDirectoryTrait};
use crate::utils::progress_bar::ProgressBarWrapper;

pub trait EnvironmentOriginalMeshesModuleBuilders : Sized {
    fn build_from_environment_creator(s: &ResourcesSingleEnvironmentDirectory, environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String>;
}
impl EnvironmentOriginalMeshesModuleBuilders for ApolloEnvironmentOriginalMeshesModule {
    fn build_from_environment_creator(s: &ResourcesSingleEnvironmentDirectory, environment_creator: &ApolloEnvironmentCreator) -> Result<Self, String> {
        let mut out = Self {
            0: ApolloOriginalMeshesModule { link_mesh_relative_paths: vec![] },
        };
        out.0.link_mesh_relative_paths.push(None);

        environment_creator.actions.iter().for_each(|action| {
            match action {
                EnvironmentCreatorAction::AddAlreadyExistingEnvironment { name, .. } => {
                    let r = ResourcesEnvironmentsDirectory::new(s.environments_directory.clone());
                    let ss = r.get_subdirectory(name);
                    let original_meshes_module = ApolloEnvironmentOriginalMeshesModule::load_or_build(&ss, false).expect("error");
                    let description_module = ApolloEnvironmentDescriptionModule::load_or_build(&ss, false).expect("error");

                    description_module.urdf_module.links.iter().enumerate().for_each(|(i, x)| {
                        if x.name != "world_environment_origin" {
                            out.0.link_mesh_relative_paths.push(original_meshes_module.0.link_mesh_relative_paths[i].clone());
                        }
                    });
                }
                EnvironmentCreatorAction::AddSingleObjectFromStlFile { fp, .. } => {
                    let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                    let target = Self::full_path_to_module_dir(s).append("meshes").append(&filename);
                    if !target.exists() {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&target);
                        let relative_file_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes").append(&filename);
                        out.0.link_mesh_relative_paths.push(Some(relative_file_path));
                    }
                }
                EnvironmentCreatorAction::AddSingleObjectFromObjFile { fp, .. } => {
                    let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                    let target = Self::full_path_to_module_dir(s).append("meshes").append(&filename);
                    if !target.exists() {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&target);
                        let relative_file_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes").append(&filename);
                        out.0.link_mesh_relative_paths.push(Some(relative_file_path));
                    }
                }
                EnvironmentCreatorAction::AddSingleObjectFromGlbFile { fp, .. } => {
                    let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                    let target = Self::full_path_to_module_dir(s).append("meshes").append(&filename);
                    if !target.exists() {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&target);
                        let relative_file_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes").append(&filename);
                        out.0.link_mesh_relative_paths.push(Some(relative_file_path));
                    }
                }
                EnvironmentCreatorAction::AddSceneFromGlbFile { scene_name, fp, .. } => {
                    let scene_path = s.directory.clone().append("mesh_modules/glb_scenes").append(scene_name).append("scene.glb");
                    let mesh_object_scene = if scene_path.exists() {
                        load_gltf_file(&scene_path).expect("error").to_mesh_object_scene()
                    } else {
                        assert!(fp.exists());
                        fp.copy_file_to_destination_file_path(&scene_path);
                        load_gltf_file(fp).expect("error").to_mesh_object_scene()
                    };

                    mesh_object_scene.nodes.iter().for_each(|node| {
                        let path = Self::full_path_to_module_dir(s).append("meshes").append(&node.name).append_without_separator(".glb");
                        node.local_space_mesh_object.save_to_glb(&path);
                        let relative_file_path = Self::relative_file_path_from_root_dir_to_module_dir(s).append("meshes").append(&node.name).append_without_separator(".glb");
                        out.0.link_mesh_relative_paths.push(Some(relative_file_path));
                    });
                }
                _ => { }

            }
        });

        Ok(out)
    }
}

impl PreprocessorModule for ApolloEnvironmentOriginalMeshesModule {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/original_meshes_module".to_string()
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
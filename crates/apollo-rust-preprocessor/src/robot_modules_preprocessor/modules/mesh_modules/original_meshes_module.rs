use crate::robot_modules_preprocessor::AdjustedRobot;
use crate::robot_modules_preprocessor::{ApolloChainCreator, ChainCreatorAction, CombinedRobot};
use crate::utils::progress_bar::ProgressBarWrapper;
use crate::{
    create_generic_build_from_adjusted_robot, create_generic_build_from_combined_robot,
    PreprocessorModule,
};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::gltf::load_gltf_file;
use apollo_rust_mesh_utils::mesh_object_scene::ToMeshObjectScene;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::{ApolloURDFGeometry, ApolloURDFModule};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::ResourcesSubDirectory;
use std::path::PathBuf;
pub trait OriginalMeshesModuleBuilders<P: ApolloPathBufTrait + Clone>: Sized {
    fn build_from_urdf_module(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
    fn build_from_combined_robot(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
    fn build_from_adjusted_robot(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
    fn build_from_chain_creator(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String>;
}
impl<P: ApolloPathBufTrait + Clone> OriginalMeshesModuleBuilders<P>
    for ApolloOriginalMeshesModule<P>
{
    fn build_from_urdf_module(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false);

        return if let Ok(urdf_module) = urdf_module {
            let mut out = vec![];

            let num_links = urdf_module.links.len();
            urdf_module.links.iter().enumerate().for_each(|(i, x)| {
                let progress = i as f64 / num_links as f64;
                progress_bar.update_with_percentage_preset(progress * 100.0);
                if x.visual.len() > 0 {
                    // let visual = &x.visual[0];
                    let mut curr_out = vec![];
                    for visual in &x.visual {
                        match &visual.geometry {
                            ApolloURDFGeometry::Mesh { filename, scale: _ } => {
                                println!("\n {:?}", filename);
                                let fp = P::new_from_str(filename);
                                let filename = fp
                                    .extract_last_n_segments(1)
                                    .split_into_strings()
                                    .pop()
                                    .unwrap();
                                let relative_path =
                                    Self::relative_file_path_from_root_dir_to_module_dir(s)
                                        .append("meshes")
                                        .append(&filename);
                                let target = Self::full_path_to_module_dir(s)
                                    .append("meshes")
                                    .append(&filename);

                                if fp.path_exists() {
                                    fp.copy_file_to_destination_file_path(&target);
                                    curr_out.push(relative_path);
                                    continue;
                                }

                                // if !target.exists() {
                                let mut curr = 7;
                                'l: loop {
                                    let ff = fp.extract_last_n_segments(curr);
                                    println!(
                                        "searching directory for file that ends with {:?}",
                                        ff
                                    );
                                    // Walk directory might be hard generic. Let's skip for now or use native PathBuf if we are building.
                                    // Actually, we can just use native PathBuf if we are in the preprocessor building phase.
                                    // But the preprocessor itself should be generic.

                                    // For now, let's just use to_path_buf for desktop walk
                                    let find = PathBuf::new_from_desktop_dir()
                                        .walk_directory_and_find_first_result(ff.to_path_buf());
                                    match find {
                                        Ok(find) => {
                                            println!("found!  Copying file.");
                                            let find_p = P::new_from_path(&find);
                                            find_p.copy_file_to_destination_file_path(&target);
                                            break 'l;
                                        }
                                        Err(_) => {
                                            if curr == 2 {
                                                panic!(
                                                    "{}",
                                                    format!(
                                                        "mesh file {:?} could not be found",
                                                        fp
                                                    )
                                                )
                                            }
                                            curr -= 1;
                                            continue 'l;
                                        }
                                    }
                                }
                                // }
                                curr_out.push(relative_path);
                            }
                            _ => {}
                        }
                    }
                    out.push(curr_out);
                } else {
                    out.push(vec![]);
                }
            });

            progress_bar.done_preset();
            Ok(ApolloOriginalMeshesModule {
                link_mesh_relative_paths: out,
            })
        } else {
            Err("could not build from urdf module".to_string())
        };
    }

    create_generic_build_from_combined_robot!(ApolloOriginalMeshesModule, P, vec![]);

    create_generic_build_from_adjusted_robot!(ApolloOriginalMeshesModule, P);

    fn build_from_chain_creator(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let p = s.directory.clone().append("creator_module/module.json");
        let creator_module = p.load_object_from_json_file_result::<ApolloChainCreator>();

        return if let Ok(chain_creator) = creator_module {
            let mut out = ApolloOriginalMeshesModule {
                link_mesh_relative_paths: vec![],
            };
            out.link_mesh_relative_paths.push(vec![]);

            chain_creator
                .actions
                .iter()
                .for_each(|action| match action {
                    ChainCreatorAction::AddAlreadyExistingChain { name, .. } => {
                        let r =
                            ResourcesRootDirectory::new(s.root_directory.clone(), s.resources_type);
                        let ss = r.get_subdirectory(name);
                        let original_meshes_module =
                            ApolloOriginalMeshesModule::load_or_build(&ss, false).expect("error");
                        let urdf_module =
                            ApolloURDFModule::load_or_build(&ss, false).expect("error");

                        urdf_module.links.iter().enumerate().for_each(|(i, x)| {
                            if x.name != "world_environment_origin" {
                                out.link_mesh_relative_paths.push(
                                    original_meshes_module.link_mesh_relative_paths[i].clone(),
                                );
                            }
                        });
                    }
                    ChainCreatorAction::AddSingleLinkFromStlFile { fp, .. } => {
                        let fp_p = P::new_from_path(&fp);
                        let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                        let target = Self::full_path_to_module_dir(s)
                            .append("meshes")
                            .append(&filename);
                        if !target.path_exists() {
                            assert!(fp_p.path_exists());
                            fp_p.copy_file_to_destination_file_path(&target);
                            let relative_file_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes")
                                    .append(&filename);
                            out.link_mesh_relative_paths.push(vec![relative_file_path]);
                        }
                    }
                    ChainCreatorAction::AddSingleLinkFromObjFile { fp, .. } => {
                        let fp_p = P::new_from_path(&fp);
                        let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                        let target = Self::full_path_to_module_dir(s)
                            .append("meshes")
                            .append(&filename);
                        if !target.path_exists() {
                            assert!(fp_p.path_exists());
                            fp_p.copy_file_to_destination_file_path(&target);
                            let relative_file_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes")
                                    .append(&filename);
                            out.link_mesh_relative_paths.push(vec![relative_file_path]);
                        }
                    }
                    ChainCreatorAction::AddSingleLinkFromGlbFile { fp, .. } => {
                        let fp_p = P::new_from_path(&fp);
                        let filename = fp.file_name().unwrap().to_str().unwrap().to_string();
                        let target = Self::full_path_to_module_dir(s)
                            .append("meshes")
                            .append(&filename);
                        if !target.path_exists() {
                            assert!(fp_p.path_exists());
                            fp_p.copy_file_to_destination_file_path(&target);
                            let relative_file_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes")
                                    .append(&filename);
                            out.link_mesh_relative_paths.push(vec![relative_file_path]);
                        }
                    }
                    ChainCreatorAction::AddSceneFromGlbFile { scene_name, fp, .. } => {
                        let fp_p = P::new_from_path(&fp);
                        let scene_path = s
                            .directory
                            .clone()
                            .append("mesh_modules/glb_scenes")
                            .append(scene_name)
                            .append("scene.glb");
                        let mesh_object_scene = if scene_path.path_exists() {
                            load_gltf_file(&scene_path)
                                .expect("error")
                                .to_mesh_object_scene()
                        } else {
                            assert!(fp_p.path_exists());
                            fp_p.copy_file_to_destination_file_path(&scene_path);
                            load_gltf_file(&fp_p).expect("error").to_mesh_object_scene()
                        };

                        mesh_object_scene.nodes.iter().for_each(|node| {
                            let path = Self::full_path_to_module_dir(s)
                                .append("meshes")
                                .append(&node.name)
                                .append_without_separator(".glb");
                            node.local_space_mesh_object
                                .save_to_glb(&path.to_path_buf());
                            let relative_file_path =
                                Self::relative_file_path_from_root_dir_to_module_dir(s)
                                    .append("meshes")
                                    .append(&node.name)
                                    .append_without_separator(".glb");
                            out.link_mesh_relative_paths.push(vec![relative_file_path]);
                        });
                    }
                    _ => {}
                });

            progress_bar.done_preset();
            Ok(out)
        } else {
            Err("could not build from creator module".to_string())
        };
    }
}

impl<P: ApolloPathBufTrait + Clone> PreprocessorModule<P> for ApolloOriginalMeshesModule<P> {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "mesh_modules/original_meshes_module".to_string()
    }

    fn current_version() -> String {
        "0.0.2".to_string()
    }

    fn build_raw(
        s: &ResourcesSubDirectory<P>,
        progress_bar: &mut ProgressBarWrapper,
    ) -> Result<Self, String> {
        let res = Self::build_from_chain_creator(s, progress_bar);
        if let Ok(res) = res {
            return Ok(res);
        }

        let res = Self::build_from_adjusted_robot(s, progress_bar);
        if let Ok(res) = res {
            return Ok(res);
        }

        let res = Self::build_from_combined_robot(s, progress_bar);
        if let Ok(res) = res {
            return Ok(res);
        }

        return Self::build_from_urdf_module(s, progress_bar);
    }

    // create_generic_build_raw!(Self, build_from_urdf_module);
}

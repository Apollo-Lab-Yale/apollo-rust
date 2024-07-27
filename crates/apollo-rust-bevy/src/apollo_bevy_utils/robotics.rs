use std::path::PathBuf;
use bevy::asset::AssetServer;
use bevy::prelude::{Commands, Component, Res};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::V;
use apollo_rust_robotics::{PlainMeshesModuleGetFullPaths, Robot};
use crate::apollo_bevy_utils::gltf::spawn_gltf;

pub fn spawn_robot_meshes(robot_instance_idx: usize,
                          robot: &Robot,
                          state: &V,
                          path_to_bevy_assets: &PathBuf,
                          commands: &mut Commands,
                          asset_server: &Res<AssetServer>) {
    let plain_meshes_module = robot.plain_meshes_module();
    let full_paths = plain_meshes_module.get_glb_full_paths(robot.single_robot_directory());

    let fk_res = robot.fk(state);

    full_paths.iter().enumerate().for_each(|(link_idx, x)| {
        match x {
            None => { }
            Some(path) => {
                let path = path_to_bevy_assets.get_a_to_b_path(path);
                let pose = &fk_res[link_idx];
                let entity = spawn_gltf(path.clone(), Some(pose), commands, asset_server);
                let mut ec = commands.get_entity(entity).unwrap();
                ec.insert(RobotLinkMesh {
                    robot_instance_idx,
                    link_idx,
                });
            }
        }
    });
}

#[derive(Clone, Debug, Component)]
pub struct RobotLinkMesh {
    pub robot_instance_idx: usize,
    pub link_idx: usize
}
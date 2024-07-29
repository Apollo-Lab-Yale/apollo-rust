use std::path::PathBuf;
use bevy::asset::{Assets, AssetServer};
use bevy::color::Color;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{Commands, Component, Entity, Res, ResMut};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::V;
use apollo_rust_preprocessor::robot_modules_preprocessor::modules::mesh_modules::convex_decomposition_meshes_module::ConvexDecompositionMeshesModuleGetFullPaths;
use apollo_rust_preprocessor::robot_modules_preprocessor::modules::mesh_modules::convex_hull_meshes_module::ConvexHullMeshesModuleGetFullPaths;
use apollo_rust_preprocessor::robot_modules_preprocessor::modules::mesh_modules::plain_meshes_module::PlainMeshesModuleGetFullPaths;
use apollo_rust_preprocessor::robot_modules_preprocessor::modules::mesh_modules::VecOfPathBufOptionsToVecOfVecTrait;
use apollo_rust_robotics::{Robot};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::gltf::spawn_gltf;
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::obj::spawn_obj;

pub fn spawn_robot_meshes_generic(robot_instance_idx: usize,
                                  mesh_type: MeshType,
                                  full_paths: Vec<Vec<PathBuf>>,
                                  fk_res: &Vec<ISE3q>,
                                  path_to_bevy_assets: &PathBuf,
                                  commands: &mut Commands,
                                  asset_server: &Res<AssetServer>,
                                  materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
    let mut out = vec![];

    full_paths.iter().enumerate().for_each(|(link_idx, x)| {
        let mut tmp = vec![];
        x.iter().for_each(|path| {
            let path = path_to_bevy_assets.get_a_to_b_path(path);
            let pose = &fk_res[link_idx];

            let entity = match mesh_type {
                MeshType::GLB => { spawn_gltf(path.clone(), Some(pose), commands, asset_server) }
                MeshType::OBJ => { spawn_obj(path.clone(), Color::srgba(0.3, 0.3, 0.3, 0.5), Some(pose), commands, asset_server, materials) }
            };

            tmp.push(entity.clone());
            let mut ec = commands.get_entity(entity).unwrap();
            ec.insert(RobotLinkMesh {
                robot_instance_idx,
                link_idx,
            });
        });
        out.push(tmp);
    });

    out
}

pub fn spawn_robot_meshes(robot_instance_idx: usize,
                          robot_meshes_representation: RobotMeshesRepresentation,
                          mesh_type: MeshType,
                          robot: &Robot,
                          state: &V,
                          path_to_bevy_assets: &PathBuf,
                          commands: &mut Commands,
                          asset_server: &Res<AssetServer>,
                          materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
    let full_paths = match &mesh_type {
        MeshType::GLB => {
            match &robot_meshes_representation {
                RobotMeshesRepresentation::Plain => { robot.plain_meshes_module().get_glb_full_paths(robot.single_robot_directory()).to_vec_of_vec_path_bufs() }
                RobotMeshesRepresentation::ConvexHull => { robot.convex_hull_meshes_module().get_glb_full_paths(robot.single_robot_directory()).to_vec_of_vec_path_bufs() }
                RobotMeshesRepresentation::ConvexDecomposition => { robot.convex_decomposition_meshes_module().get_glb_full_paths(robot.single_robot_directory()) }
            }
        }
        MeshType::OBJ => {
            match &robot_meshes_representation {
                RobotMeshesRepresentation::Plain => { robot.plain_meshes_module().get_obj_full_paths(robot.single_robot_directory()).to_vec_of_vec_path_bufs() }
                RobotMeshesRepresentation::ConvexHull => { robot.convex_hull_meshes_module().get_obj_full_paths(robot.single_robot_directory()).to_vec_of_vec_path_bufs() }
                RobotMeshesRepresentation::ConvexDecomposition => { robot.convex_decomposition_meshes_module().get_obj_full_paths(robot.single_robot_directory()) }
            }
        }
    };

    let fk_res = robot.fk(state);

    let res = spawn_robot_meshes_generic(robot_instance_idx, mesh_type, full_paths, &fk_res, path_to_bevy_assets, commands, asset_server, materials);

    match &robot_meshes_representation {
        RobotMeshesRepresentation::Plain => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(RobotLinkPlainMesh); } ); }
        RobotMeshesRepresentation::ConvexHull => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(RobotLinkConvexHullMesh); } ); }
        RobotMeshesRepresentation::ConvexDecomposition => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(RobotLinkConvexDecompositionMesh); } ); }
    }

    res
}

#[derive(Clone, Debug, Copy)]
pub enum RobotMeshesRepresentation {
    Plain,
    ConvexHull,
    ConvexDecomposition
}

#[derive(Clone, Debug, Component)]
pub struct RobotLinkMesh {
    pub robot_instance_idx: usize,
    pub link_idx: usize
}

#[derive(Clone, Debug, Component)]
pub struct RobotLinkPlainMesh;

#[derive(Clone, Debug, Component)]
pub struct RobotLinkConvexHullMesh;

#[derive(Clone, Debug, Component)]
pub struct RobotLinkConvexDecompositionMesh;
use std::borrow::Cow;
use std::path::PathBuf;
use std::sync::Arc;
use bevy::asset::{Assets, AssetServer};
use bevy::color::Color;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{Commands, Component, Entity, Query, Res, ResMut, Transform};
use bevy_egui::egui::{Slider, Ui};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_linalg::{V};
use apollo_rust_robotics_core::Chain;
use apollo_rust_robotics_core::modules::mesh_modules::convex_decomposition_meshes_module::ConvexDecompositionMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::convex_hulls_meshes_module::ConvexHullMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::plain_meshes_module::PlainMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::VecOfPathBufOptionsToVecOfVecTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::gltf::spawn_gltf;
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::obj::spawn_obj;
use crate::apollo_bevy_utils::transform::TransformUtils;

#[derive(Clone)]
pub struct BevySpawnChainMeshes {
    pub chain_instance_idx: usize,
    pub chain_meshes_representation: ChainMeshesRepresentation,
    pub mesh_type: MeshType,
    pub chain: Arc<Chain>,
    pub path_to_bevy_assets: PathBuf,
    pub state: V
}
impl BevySpawnChainMeshes {
    fn spawn_chain_meshes_generic(&self,
                                  full_paths: &Vec<Vec<PathBuf>>,
                                  fk_res: &Vec<ISE3q>,
                                  commands: &mut Commands,
                                  asset_server: &Res<AssetServer>,
                                  materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
        let mut out = vec![];

        full_paths.iter().enumerate().for_each(|(link_idx, x)| {
            let mut tmp = vec![];
            x.iter().for_each(|path| {
                let path = self.path_to_bevy_assets.get_a_to_b_path(path);
                let pose = &fk_res[link_idx];

                let entity = match self.mesh_type {
                    MeshType::GLB => { spawn_gltf(path.clone(), Some(pose), commands, asset_server) }
                    MeshType::OBJ => { spawn_obj(path.clone(), Color::srgba(0.6, 0.6, 0.62, 1.0), Some(pose), commands, asset_server, materials) }
                };
                tmp.push(entity.clone());
                let mut ec = commands.get_entity(entity).unwrap();
                ec.insert(ChainLinkMesh {
                    chain_instance_idx: self.chain_instance_idx,
                    link_idx,
                });
            });
            out.push(tmp);
        });

        out
    }

    pub fn action_spawn_chain_meshes(&self,
                                     commands: &mut Commands,
                                     asset_server: &Res<AssetServer>,
                                     materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
        // let self_clone = self.clone();

        let full_paths = match &self.mesh_type {
            MeshType::GLB => {
                match &self.chain_meshes_representation {
                    ChainMeshesRepresentation::Plain => { self.chain.plain_meshes_module().get_glb_full_paths(self.chain.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.chain.convex_hull_meshes_module().get_glb_full_paths(self.chain.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.chain.convex_decomposition_meshes_module().get_glb_full_paths(self.chain.resources_sub_directory()) }
                }
            }
            MeshType::OBJ => {
                match &self.chain_meshes_representation {
                    ChainMeshesRepresentation::Plain => { self.chain.plain_meshes_module().get_obj_full_paths(self.chain.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.chain.convex_hull_meshes_module().get_obj_full_paths(self.chain.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.chain.convex_decomposition_meshes_module().get_obj_full_paths(self.chain.resources_sub_directory()) }
                }
            }
        };

        let fk_res = self.chain.fk(&self.state);

        // let res = spawn_chain_meshes_generic(self.chain_instance_idx, self.mesh_type, full_paths, &fk_res, &self.path_to_bevy_assets, commands, asset_server, materials);

        let res = self.clone().spawn_chain_meshes_generic(&full_paths, &fk_res, commands, asset_server, materials);

        match &self.chain_meshes_representation {
            ChainMeshesRepresentation::Plain => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkPlainMesh); }); }
            ChainMeshesRepresentation::ConvexHull => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkConvexHullMesh); }); }
            ChainMeshesRepresentation::ConvexDecomposition => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkConvexDecompositionMesh); }); }
        }

        res
    }

    pub fn get_system(self) -> impl FnMut(Commands, Res<AssetServer>, ResMut<Assets<StandardMaterial>>) + 'static {
        let c: Cow<'_, BevySpawnChainMeshes> = Cow::Owned(self.clone());
        move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
            c.as_ref().action_spawn_chain_meshes(&mut commands, &asset_server, &mut materials);
        }
    }
}

/*
pub fn spawn_chain_meshes_generic(chain_instance_idx: usize,
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
                MeshType::OBJ => { spawn_obj(path.clone(), Color::srgba(0.6, 0.6, 0.62, 1.0), Some(pose), commands, asset_server, materials) }
            };

            tmp.push(entity.clone());
            let mut ec = commands.get_entity(entity).unwrap();
            ec.insert(ChainLinkMesh {
                chain_instance_idx: chain_instance_idx,
                link_idx,
            });
        });
        out.push(tmp);
    });

    out
}

pub fn spawn_chain_meshes(chain_instance_idx: usize,
                          chain_meshes_representation: ChainMeshesRepresentation,
                          mesh_type: MeshType,
                          robot: &Chain,
                          state: &V,
                          path_to_bevy_assets: &PathBuf,
                          commands: &mut Commands,
                          asset_server: &Res<AssetServer>,
                          materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
    let full_paths = match &mesh_type {
        MeshType::GLB => {
            match &chain_meshes_representation {
                ChainMeshesRepresentation::Plain => { robot.plain_meshes_module().get_glb_full_paths(robot.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                ChainMeshesRepresentation::ConvexHull => { robot.convex_hull_meshes_module().get_glb_full_paths(robot.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                ChainMeshesRepresentation::ConvexDecomposition => { robot.convex_decomposition_meshes_module().get_glb_full_paths(robot.resources_sub_directory()) }
            }
        }
        MeshType::OBJ => {
            match &chain_meshes_representation {
                ChainMeshesRepresentation::Plain => { robot.plain_meshes_module().get_obj_full_paths(robot.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                ChainMeshesRepresentation::ConvexHull => { robot.convex_hull_meshes_module().get_obj_full_paths(robot.resources_sub_directory()).to_vec_of_vec_path_bufs() }
                ChainMeshesRepresentation::ConvexDecomposition => { robot.convex_decomposition_meshes_module().get_obj_full_paths(robot.resources_sub_directory()) }
            }
        }
    };

    let fk_res = robot.fk(state);

    let res = spawn_chain_meshes_generic(chain_instance_idx, mesh_type, full_paths, &fk_res, path_to_bevy_assets, commands, asset_server, materials);

    match &chain_meshes_representation {
        ChainMeshesRepresentation::Plain => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkPlainMesh); } ); }
        ChainMeshesRepresentation::ConvexHull => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkConvexHullMesh); } ); }
        ChainMeshesRepresentation::ConvexDecomposition => { res.iter().flatten().for_each(|x| { commands.entity(x.clone()).insert(ChainLinkConvexDecompositionMesh); } ); }
    }

    res
}
*/

pub struct BevyChainSlidersEgui<'a> {
    pub chain_instance_idx: usize,
    pub chain: Arc<Chain>,
    pub ui: &'a mut Ui
}
impl<'a> BevyChainSlidersEgui<'a> {
    pub fn action_chain_sliders_egui(&'a mut self, chain_state: &mut ChainState) {
        let dof_module = self.chain.dof_module();
        let bounds_module = self.chain.bounds_module();
        let num_dofs = dof_module.num_dofs;

        let robot_state = &mut chain_state.state;

        for dof_idx in 0..num_dofs {
            let bounds = bounds_module.bounds[dof_idx];
            self.ui.horizontal(|ui| {
                ui.label(format!("{}: ", dof_idx));
                ui.add(Slider::new(&mut robot_state[dof_idx], bounds.0..=bounds.1));
            });
        }
    }
}

/*
pub fn chain_sliders_egui(chain_instance_idx: usize, robot: &Chain, ui: &mut Ui, chain_states: &mut ResMut<ChainStates>) {
    let dof_module = robot.dof_module();
    let bounds_module = robot.bounds_module();
    let num_dofs = dof_module.num_dofs;

    let robot_state = chain_states.states.get_mut(chain_instance_idx).expect("error");

    for dof_idx in 0..num_dofs {
        let bounds = bounds_module.bounds[dof_idx];
        ui.horizontal(|ui| {
            ui.label(format!("{}: ", dof_idx));
            ui.add(Slider::new(&mut robot_state[dof_idx], bounds.0..=bounds.1));
        });
    }
}
*/

#[derive(Clone)]
pub struct BevyChainStateUpdaterLoop {
    pub chain_instance_idx: usize,
    pub chain: Arc<Chain>
}
impl BevyChainStateUpdaterLoop {
    pub fn action_pose_chain(chain_instance_idx: usize, chain: &Chain, state: &V, query: &mut Query<(&mut Transform, &ChainLinkMesh)>) {
        let fk_res = chain.fk(state);
        query.iter_mut().for_each(|(mut x, y)| {
            if y.chain_instance_idx == chain_instance_idx {
                let link_idx = y.link_idx;
                *x = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&fk_res[link_idx]);
            }
        });
    }

    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh)>, Query<&mut ChainState>) + 'static {
        let c: Cow<'_, BevyChainStateUpdaterLoop> = Cow::Owned(self.clone());
        return move | mut query1:  Query<(&mut Transform, &ChainLinkMesh)>, query2: Query<&mut ChainState>| {
            for x in query2.iter() {
                if x.chain_instance_idx == self.chain_instance_idx {
                    Self::action_pose_chain(c.as_ref().chain_instance_idx, &c.as_ref().chain, &x.state, &mut query1);
                }
            }
        }
    }

    /*
    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh)>, Res<ChainStates>) + 'static {
        let c: Cow<'_, BevyChainStateUpdaterLoop> = Cow::Owned(self.clone());
        return move |mut query: Query<(&mut Transform, &ChainLinkMesh)>, chain_states: Res<ChainStates>| {
            if chain_states.is_changed() {
                chain_states.states.iter().enumerate().for_each(|(i, x)| {
                    bevy_pose_chain(i, &c.as_ref().chain, x, &mut query);
                });
            }
        }
    }
    */
}

/*
pub fn chain_state_updater_loop(chain: &Chain, query: &mut Query<(&mut Transform, &ChainLinkMesh)>, chain_states: &Res<ChainStates>) {
    if chain_states.is_changed() {
        chain_states.states.iter().enumerate().for_each(|(i, x)| {
            bevy_pose_chain(i, chain, x, query);
        });
    }
}
*/

/*
#[derive(Resource)]
pub struct ChainStates {
    pub states: Vec<V>
}
*/

#[derive(Component)]
pub struct ChainState {
    pub chain_instance_idx: usize,
    pub state: V
}

#[derive(Clone, Debug, Copy)]
pub enum ChainMeshesRepresentation {
    Plain,
    ConvexHull,
    ConvexDecomposition
}

#[derive(Clone, Debug, Component)]
pub struct ChainLinkMesh {
    pub chain_instance_idx: usize,
    pub link_idx: usize
}

#[derive(Clone, Debug, Component)]
pub struct ChainLinkPlainMesh;

#[derive(Clone, Debug, Component)]
pub struct ChainLinkConvexHullMesh;

#[derive(Clone, Debug, Component)]
pub struct ChainLinkConvexDecompositionMesh;
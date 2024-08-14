use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use bevy::asset::{Assets, AssetServer};
use bevy::color::Color;
use bevy::pbr::StandardMaterial;
use bevy::prelude::{Changed, Commands, Component, Entity, Query, Res, ResMut, Transform, Window as Window1, With};
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{SidePanel, Slider, Ui};
use bevy_egui::EguiContexts;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_linalg::{V};
use apollo_rust_robotics_core::Chain;
use apollo_rust_robotics_core::modules::mesh_modules::convex_decomposition_meshes_module::ConvexDecompositionMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::convex_hulls_meshes_module::ConvexHullMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::plain_meshes_module::PlainMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::VecOfPathBufOptionsToVecOfVecTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::apollo_bevy_utils::colors::BaseColor;
use crate::apollo_bevy_utils::egui::{CursorIsOverEgui, set_cursor_is_over_egui_default};
use crate::apollo_bevy_utils::gltf::spawn_gltf;
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::obj::spawn_obj;
use crate::apollo_bevy_utils::signatures::{ChainMeshComponents, Signature, Signatures};
use crate::apollo_bevy_utils::transform::TransformUtils;
use crate::apollo_bevy_utils::visibility::BaseVisibility;

#[derive(Clone)]
pub struct BevySpawnChainMeshes {
    pub chain_instance_idx: usize,
    pub chain_meshes_representation: ChainMeshesRepresentation,
    pub mesh_type: MeshType,
    pub chain: Arc<Chain>,
    pub path_to_bevy_assets: PathBuf,
    pub state: V,
    pub base_visibility_mode: BaseVisibility
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
                commands.entity(entity).insert(ChainLinkMesh {
                    chain_instance_idx: self.chain_instance_idx,
                    link_idx,
                });
                commands.entity(entity).insert(self.base_visibility_mode.clone());
                commands.entity(entity).insert(BaseColor(Color::srgba(0.6, 0.6, 0.62, 1.0)));
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
            ChainMeshesRepresentation::Plain => {
                res.iter().flatten().for_each(|x| {
                    commands.entity(x.clone()).insert(ChainLinkPlainMesh);
                });
            }
            ChainMeshesRepresentation::ConvexHull => {
                res.iter().flatten().for_each(|x| {
                    commands.entity(x.clone()).insert(ChainLinkConvexHullMesh);
                });
            }
            ChainMeshesRepresentation::ConvexDecomposition => {
                res.iter().flatten().for_each(|x| {
                    commands.entity(x.clone()).insert(ChainLinkConvexDecompositionMesh);
                });
            }
        }

        res.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(i, y)| {
                let mut hm = HashMap::new();

                let p = ChainMeshComponents::get_power_set(self.chain_meshes_representation.clone(), self.mesh_type, self.chain_instance_idx, link_idx, i);
                for pp in p {
                    hm.insert(Signature::ChainLinkMesh { components: pp.clone() }, () );
                }

                commands.entity(y.clone()).insert(Signatures(hm));
            });
        });

        res
    }

    pub fn get_system(self) -> impl FnMut(Commands, Res<AssetServer>, ResMut<Assets<StandardMaterial>>) + 'static {
        move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
            self.action_spawn_chain_meshes(&mut commands, &asset_server, &mut materials);
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

pub struct BevyChainSlidersEgui {
    pub chain_instance_idx: usize,
    pub chain: Arc<Chain>,
}
impl BevyChainSlidersEgui {
    pub fn action_chain_sliders_egui(&self, query: &mut Query<&mut ChainState>, ui: &mut Ui) {
        query.iter_mut().for_each(|mut x| {
            if x.chain_instance_idx == self.chain_instance_idx {
                let dof_module = self.chain.dof_module();
                let bounds_module = self.chain.bounds_module();
                let num_dofs = dof_module.num_dofs;

                let robot_state = &mut x.state;

                for dof_idx in 0..num_dofs {
                    let bounds = bounds_module.bounds[dof_idx];
                    ui.horizontal(|ui| {
                        ui.label(format!("{}: ", dof_idx));
                        ui.add(Slider::new(&mut robot_state[dof_idx], bounds.0..=bounds.1));
                    });
                }
            }
        });
    }

    pub fn get_system_side_panel_left(self) -> impl FnMut(EguiContexts, Query<&mut ChainState>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>) + 'static {
        return move |mut egui_contexts: EguiContexts, mut query: Query<&mut ChainState>, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window1, With<PrimaryWindow>>| {
            SidePanel::left(format!("chain_sliders_side_panel_chain_instance_idx_{}", self.chain_instance_idx))
                .show(egui_contexts.ctx_mut(), |ui| {
                    self.action_chain_sliders_egui(&mut query, ui);
                    set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
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
    pub fn action_pose_chain(chain_instance_idx: usize, chain: &Chain, state: &V, global_offset: &ISE3q, query: &mut Query<(&mut Transform, &ChainLinkMesh)>) {
        let fk_res = chain.fk(state);
        query.iter_mut().for_each(|(mut x, y)| {
            if y.chain_instance_idx == chain_instance_idx {
                let link_idx = y.link_idx;
                *x = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&global_offset.group_operator(&fk_res[link_idx]));
            }
        });
    }

    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh)>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        return move | mut query1:  Query<(&mut Transform, &ChainLinkMesh)>, query2: Query<&mut ChainState, Changed<ChainState>>| {
            for x in query2.iter() {
                if x.chain_instance_idx == self.chain_instance_idx {
                    Self::action_pose_chain(self.chain_instance_idx, &self.chain, &x.state, &x.global_offset, &mut query1);
                }
            }
        }
    }
}

#[derive(Component)]
pub struct ChainState {
    pub chain_instance_idx: usize,
    pub state: V,
    pub global_offset: ISE3q
}

#[derive(Clone, Debug, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
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
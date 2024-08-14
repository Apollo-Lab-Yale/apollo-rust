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
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_robotics_core::modules::mesh_modules::convex_decomposition_meshes_module::ConvexDecompositionMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::convex_hulls_meshes_module::ConvexHullMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::plain_meshes_module::PlainMeshesModuleGetFullPaths;
use apollo_rust_robotics_core::modules::mesh_modules::VecOfPathBufOptionsToVecOfVecTrait;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
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
pub struct BevySpawnChainMeshesRaw {
    pub chain_instance_idx: usize,
    pub chain_meshes_representation: ChainMeshesRepresentation,
    pub mesh_type: MeshType,
    pub resources_sub_directory: ResourcesSubDirectory,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub plain_meshes_module: ApolloPlainMeshesModule,
    pub convex_hull_meshes_module: ApolloConvexHullMeshesModule,
    pub convex_decomposition_meshes_module: ApolloConvexDecompositionMeshesModule,
    pub path_to_bevy_assets: PathBuf,
    pub state: V,
    pub base_visibility_mode: BaseVisibility
}
impl BevySpawnChainMeshesRaw {
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
                    ChainMeshesRepresentation::Plain => { self.plain_meshes_module.get_glb_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.convex_hull_meshes_module.get_glb_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.convex_decomposition_meshes_module.get_glb_full_paths(&self.resources_sub_directory) }
                }
            }
            MeshType::OBJ => {
                match &self.chain_meshes_representation {
                    ChainMeshesRepresentation::Plain => { self.plain_meshes_module.get_obj_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.convex_hull_meshes_module.get_obj_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.convex_decomposition_meshes_module.get_obj_full_paths(&self.resources_sub_directory) }
                }
            }
        };

        // let fk_res = self.chain.fk(&self.state);
        let fk_res = RobotKinematicsFunctions::fk(&self.state, &self.urdf_module, &self.chain_module, &self.dof_module);

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

#[derive(Clone)]
pub struct BevySpawnChainMeshes {
    pub chain_instance_idx: usize,
    pub chain_meshes_representation: ChainMeshesRepresentation,
    pub mesh_type: MeshType,
    pub chain: Arc<ChainNalgebra>,
    pub path_to_bevy_assets: PathBuf,
    pub state: V,
    pub base_visibility_mode: BaseVisibility
}
impl BevySpawnChainMeshes {
    /*
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
    */

    pub fn get_system(self) -> impl FnMut(Commands, Res<AssetServer>, ResMut<Assets<StandardMaterial>>) + 'static {
        let raw = BevySpawnChainMeshesRaw {
            chain_instance_idx: self.chain_instance_idx,
            chain_meshes_representation: self.chain_meshes_representation.clone(),
            mesh_type: self.mesh_type.clone(),
            resources_sub_directory: self.chain.resources_sub_directory().clone(),
            urdf_module: self.chain.urdf_module.clone(),
            chain_module: self.chain.chain_module.clone(),
            dof_module: self.chain.dof_module.clone(),
            plain_meshes_module: self.chain.plain_meshes_module.clone(),
            convex_hull_meshes_module: self.chain.convex_hull_meshes_module.clone(),
            convex_decomposition_meshes_module: self.chain.convex_decomposition_meshes_module.clone(),
            path_to_bevy_assets: self.path_to_bevy_assets.clone(),
            state: self.state.clone(),
            base_visibility_mode: self.base_visibility_mode.clone(),
        };

        /*
        move |mut commands: Commands, asset_server: Res<AssetServer>, mut materials: ResMut<Assets<StandardMaterial>>| {
            self.action_spawn_chain_meshes(&mut commands, &asset_server, &mut materials);
        }
        */

        return raw.get_system()
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct BevyChainSlidersEguiRaw {
    pub chain_instance_idx: usize,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub bounds_module: ApolloBoundsModule
}
impl BevyChainSlidersEguiRaw {
    pub fn action_chain_sliders_egui(&self, query: &mut Query<&mut ChainState>, ui: &mut Ui) {
        query.iter_mut().for_each(|mut x| {
            if x.chain_instance_idx == self.chain_instance_idx {
                let dof_module = &self.dof_module;
                let bounds_module = &self.bounds_module;
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

    pub fn get_system_side_panel_left<F: FnMut(&mut Ui, &BevyChainSlidersEguiRaw) + 'static>(self, mut f: F) -> impl FnMut(EguiContexts, Query<&mut ChainState>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>) + 'static {
        return move |mut egui_contexts: EguiContexts, mut query: Query<&mut ChainState>, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window1, With<PrimaryWindow>>| {
            let self_clone = self.clone();
            SidePanel::left(format!("chain_sliders_side_panel_chain_instance_idx_{}", self.chain_instance_idx))
                .show(egui_contexts.ctx_mut(),  |ui| {
                    self_clone.action_chain_sliders_egui(&mut query, ui);
                    f(ui, &self_clone);
                    set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
                });
        }
    }
}

#[derive(Clone)]
pub struct BevyChainSlidersEgui {
    pub chain_instance_idx: usize,
    pub chain: Arc<ChainNalgebra>,
}
impl BevyChainSlidersEgui {
    pub fn get_system_side_panel_left<F: FnMut(&mut Ui, &BevyChainSlidersEguiRaw) + 'static>(self, f: F) -> impl FnMut(EguiContexts, Query<&mut ChainState>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>) + 'static {
        let raw = BevyChainSlidersEguiRaw {
            chain_instance_idx: self.chain_instance_idx,
            urdf_module: self.chain.urdf_module.clone(),
            chain_module: self.chain.chain_module.clone(),
            dof_module: self.chain.dof_module.clone(),
            bounds_module: self.chain.bounds_module.clone(),
        };

        return raw.get_system_side_panel_left(f)
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone)]
pub struct BevyChainStateUpdaterLoopRaw {
    pub chain_instance_idx: usize,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
}
impl BevyChainStateUpdaterLoopRaw {
    pub fn action_pose_chain(&self, state: &V, global_offset: &ISE3q, query: &mut Query<(&mut Transform, &ChainLinkMesh)>) {
        // let fk_res = chain.fk(state);
        let fk_res = RobotKinematicsFunctions::fk(state, &self.urdf_module, &self.chain_module, &self.dof_module);
        query.iter_mut().for_each(|(mut x, y)| {
            if y.chain_instance_idx == self.chain_instance_idx {
                let link_idx = y.link_idx;
                *x = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&global_offset.group_operator(&fk_res[link_idx]));
            }
        });
    }

    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh)>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        return move | mut query1:  Query<(&mut Transform, &ChainLinkMesh)>, query2: Query<&mut ChainState, Changed<ChainState>>| {
            for x in query2.iter() {
                if x.chain_instance_idx == self.chain_instance_idx {
                    self.action_pose_chain(&x.state, &x.global_offset, &mut query1);
                }
            }
        }
    }
}

#[derive(Clone)]
pub struct BevyChainStateUpdaterLoop {
    pub chain_instance_idx: usize,
    pub chain: Arc<ChainNalgebra>
}
impl BevyChainStateUpdaterLoop {
    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh)>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        let raw = BevyChainStateUpdaterLoopRaw {
            chain_instance_idx: self.chain_instance_idx,
            urdf_module: self.chain.urdf_module.clone(),
            chain_module: self.chain.chain_module.clone(),
            dof_module: self.chain.dof_module.clone(),
        };
        return raw.get_system()
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

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
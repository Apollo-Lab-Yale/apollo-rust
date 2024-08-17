use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use bevy::asset::{Assets, AssetServer};
use bevy::color::Color;
use bevy::pbr::{PbrBundle, StandardMaterial};
use bevy::prelude::{Changed, Commands, Component, Cuboid, default, Entity, Mesh, Query, Res, ResMut, Sphere, Transform, Window as Window1, With, Without};
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{ScrollArea, SidePanel, Slider, Ui};
use bevy_egui::EguiContexts;
use bevy_mod_outline::{OutlineBundle, OutlineMode, OutlineVolume};
use apollo_rust_algs::VecOfOptionsToVecOfVecsTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_lie::LieGroupElement;
use apollo_rust_linalg::{V};
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::{ApolloLinkShapesApproximationsModule, BoundingSphereDescriptor, OBBDescriptor};
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
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::vectors::V3;
use crate::apollo_bevy_utils::colors::{BaseColor, ColorChangeEngine, ColorChangeRequest, ColorChangeRequestType};
use crate::apollo_bevy_utils::egui::{CursorIsOverEgui, set_cursor_is_over_egui_default};
use crate::apollo_bevy_utils::gltf::spawn_gltf;
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::obj::spawn_obj;
use crate::apollo_bevy_utils::signatures::{ChainMeshComponent, ChainMeshComponents, Signature, Signatures};
use crate::apollo_bevy_utils::transform::TransformUtils;
use crate::apollo_bevy_utils::visibility::{BaseVisibility, VisibilityChangeEngine, VisibilityChangeRequest, VisibilityChangeRequestType};

////////////////////////////////////////////////////////////////////////////////////////////////////

pub struct BevySpawnChainLinkApproximationsRaw {
    pub chain_instance_idx: usize,
    pub resources_sub_directory: ResourcesSubDirectory,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub link_shapes_approximations_module: ApolloLinkShapesApproximationsModule,
    pub state: V,
    pub base_visibility_mode: BaseVisibility
}
impl BevySpawnChainLinkApproximationsRaw {
    fn action_spawn_bounding_sphere_approximations(&self,
                                                   d: &Vec<Vec<BoundingSphereDescriptor>>,
                                                   fk_res: &Vec<ISE3q>,
                                                   commands: &mut Commands,
                                                   meshes: &mut ResMut<Assets<Mesh>>,
                                                   materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
        let mut out = vec![];
        d.iter().enumerate().for_each(|(link_idx, x)| {
            let pose = fk_res[link_idx].clone();
            let t = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&pose);
            let mut tmp = vec![];
            x.iter().enumerate().for_each(|(_subcomponent_idx, y)| {
                let mut e = commands.spawn(PbrBundle {
                    mesh: meshes.add(Mesh::from(Sphere { radius: y.radius as f32 })),
                    material: materials.add(StandardMaterial {
                        base_color: Color::srgba(0.7, 0.7, 0.8, 0.3),
                        ..Default::default()
                    }),
                    transform: t,
                    ..Default::default()
                });
                e
                    // .set_parent(id)
                    .insert(self.base_visibility_mode.clone())
                    .insert(BaseColor(Color::srgba(0.7, 0.7, 0.8, 0.3)))
                    .insert(ChainLinkMesh { chain_instance_idx: self.chain_instance_idx, link_idx })
                    .insert(LinkApproximatingShape)
                    .insert(OffsetFrame(ISE3q::new(I3::from_slices_euler_angles(&y.offset_xyz, &y.offset_rpy))))
                    .insert(OutlineBundle {
                        outline: OutlineVolume {
                            visible: true,
                            width: 1.0,
                            colour: Color::srgb(0.,0.,0.),
                        },
                        mode: OutlineMode::RealVertex,
                        ..default()
                    });

                tmp.push(e.id());
            });

            out.push(tmp);
        });

        out
    }

    fn action_spawn_obb_approximations(&self,
                                       d: &Vec<Vec<OBBDescriptor>>,
                                       fk_res: &Vec<ISE3q>,
                                       commands: &mut Commands,
                                       meshes: &mut ResMut<Assets<Mesh>>,
                                       materials: &mut ResMut<Assets<StandardMaterial>>) -> Vec<Vec<Entity>> {
        let mut out = vec![];
        d.iter().enumerate().for_each(|(link_idx, x)| {
            let pose = fk_res[link_idx].clone();
            let t = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&pose);
            let mut tmp = vec![];
            x.iter().enumerate().for_each(|(_subcomponent_idx, y)| {
                let mut e = commands.spawn(PbrBundle {
                    mesh: meshes.add(Mesh::from(Cuboid { half_size: TransformUtils::util_convert_z_up_v3_to_z_up_vec3(V3::from_column_slice(&y.half_extents)) })),
                    material: materials.add(StandardMaterial {
                        base_color: Color::srgba(0.7, 0.8, 0.7, 1.0),
                        ..Default::default()
                    }),
                    transform: t,
                    ..Default::default()
                });
                e
                    // .set_parent(id)
                    .insert(self.base_visibility_mode.clone())
                    .insert(BaseColor(Color::srgba(0.7, 0.7, 0.8, 0.5)))
                    .insert(ChainLinkMesh { chain_instance_idx: self.chain_instance_idx, link_idx })
                    .insert(LinkApproximatingShape)
                    .insert(OffsetFrame(ISE3q::new(I3::from_slices_euler_angles(&y.offset_xyz, &y.offset_rpy))))
                    .insert(OutlineBundle {
                        outline: OutlineVolume {
                            visible: true,
                            width: 1.0,
                            colour: Color::srgb(0.,0.,0.),
                        },
                        mode: OutlineMode::RealVertex,
                        ..default()
                    });

                tmp.push(e.id());
            });

            out.push(tmp);
        });

        out
    }

    pub fn action_spawn_all_chain_shape_approximations(&self,
                                                       commands: &mut Commands,
                                                       meshes: &mut ResMut<Assets<Mesh>>,
                                                       materials: &mut ResMut<Assets<StandardMaterial>>) {

        let fk_res = RobotKinematicsFunctions::fk(&self.state, &self.urdf_module, &self.chain_module, &self.dof_module);
        let d = self.link_shapes_approximations_module.full_bounding_spheres.to_vec_of_vecs();
        let e = self.action_spawn_bounding_sphere_approximations(&d, &fk_res, commands, meshes, materials);

        e.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(s, y)| {
                let original_set = vec![ ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::BoundingSphereFull), ChainMeshComponent::ChainInstanceIdx(self.chain_instance_idx), ChainMeshComponent::LinkIdx(link_idx), ChainMeshComponent::SubcomponentIdx(s) ];
                let p = ChainMeshComponents::get_power_set_general(original_set);
                let mut m = HashMap::new();
                p.iter().for_each(|x| { m.insert(Signature::ChainLinkMesh { components: x.clone() }, () ); });
                commands.entity(y.clone()).insert(Signatures(m));
            })
        });

        let d = &self.link_shapes_approximations_module.decomposition_bounding_spheres;
        let e = self.action_spawn_bounding_sphere_approximations(d, &fk_res, commands, meshes, materials);

        e.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(s, y)| {
                let original_set = vec![ ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::BoundingSphereDecomposition), ChainMeshComponent::ChainInstanceIdx(self.chain_instance_idx), ChainMeshComponent::LinkIdx(link_idx), ChainMeshComponent::SubcomponentIdx(s) ];
                let p = ChainMeshComponents::get_power_set_general(original_set);
                let mut m = HashMap::new();
                p.iter().for_each(|x| { m.insert(Signature::ChainLinkMesh { components: x.clone() }, () ); });
                commands.entity(y.clone()).insert(Signatures(m));
            })
        });

        let d = self.link_shapes_approximations_module.full_obbs.to_vec_of_vecs();
        let e = self.action_spawn_obb_approximations(&d, &fk_res, commands, meshes, materials);

        e.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(s, y)| {
                let original_set = vec![ ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::OBBFull), ChainMeshComponent::ChainInstanceIdx(self.chain_instance_idx), ChainMeshComponent::LinkIdx(link_idx), ChainMeshComponent::SubcomponentIdx(s) ];
                let p = ChainMeshComponents::get_power_set_general(original_set);
                let mut m = HashMap::new();
                p.iter().for_each(|x| { m.insert(Signature::ChainLinkMesh { components: x.clone() }, () ); });
                commands.entity(y.clone()).insert(Signatures(m));
            })
        });

        let d = &self.link_shapes_approximations_module.decomposition_obbs;
        let e = self.action_spawn_obb_approximations(&d, &fk_res, commands, meshes, materials);

        e.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(s, y)| {
                let original_set = vec![ ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::OBBDecomposition), ChainMeshComponent::ChainInstanceIdx(self.chain_instance_idx), ChainMeshComponent::LinkIdx(link_idx), ChainMeshComponent::SubcomponentIdx(s) ];
                let p = ChainMeshComponents::get_power_set_general(original_set);
                let mut m = HashMap::new();
                p.iter().for_each(|x| { m.insert(Signature::ChainLinkMesh { components: x.clone() }, () ); });
                commands.entity(y.clone()).insert(Signatures(m));
            })
        });
    }

    pub fn get_system(self) -> impl FnMut(Commands, ResMut<Assets<Mesh>>, ResMut<Assets<StandardMaterial>>) + 'static {
        move |mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>| {
            self.action_spawn_all_chain_shape_approximations(&mut commands, &mut meshes, &mut materials);
        }
    }
}

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
    pub link_shapes_approximations_module: ApolloLinkShapesApproximationsModule,
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
        let full_paths = match &self.mesh_type {
            MeshType::GLB => {
                match &self.chain_meshes_representation {
                    ChainMeshesRepresentation::Plain => { self.plain_meshes_module.get_glb_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.convex_hull_meshes_module.get_glb_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.convex_decomposition_meshes_module.get_glb_full_paths(&self.resources_sub_directory) }
                    _ => { panic!("not handled here.") }
                }
            }
            MeshType::OBJ => {
                match &self.chain_meshes_representation {
                    ChainMeshesRepresentation::Plain => { self.plain_meshes_module.get_obj_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexHull => { self.convex_hull_meshes_module.get_obj_full_paths(&self.resources_sub_directory).to_vec_of_vec_path_bufs() }
                    ChainMeshesRepresentation::ConvexDecomposition => { self.convex_decomposition_meshes_module.get_obj_full_paths(&self.resources_sub_directory) }
                    _ => { panic!("not handled here.") }
                }
            }
        };

        let fk_res = RobotKinematicsFunctions::fk(&self.state, &self.urdf_module, &self.chain_module, &self.dof_module);

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
            _ => { panic!("not handled here.") }
        }

        res.iter().enumerate().for_each(|(link_idx, x)| {
            x.iter().enumerate().for_each(|(i, y)| {
                let mut hm = HashMap::new();
                let p = ChainMeshComponents::get_power_set_default(self.chain_meshes_representation.clone(), self.mesh_type, self.chain_instance_idx, link_idx, i);
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
            link_shapes_approximations_module: self.chain.link_shapes_approximations_module.clone(),
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
    pub fn action_chain_sliders_egui_static(
        chain_instance_idx: usize,
        urdf_module: &ApolloURDFNalgebraModule,
        chain_module: &ApolloChainModule,
        dof_module: &ApolloDOFModule,
        bounds_module: &ApolloBoundsModule,
        color_change_engine: &mut ResMut<ColorChangeEngine>,
        query: &mut Query<&mut ChainState>,
        ui: &mut Ui) {
        query.iter_mut().for_each(|mut x| {
            if x.chain_instance_idx == chain_instance_idx {
                let dof_module = dof_module;
                let chain_module = chain_module;
                let joints_in_chain = &chain_module.joints_in_chain;

                let robot_state = &mut x.state;

                ui.group(|ui| {
                    ui.heading("Joints Panel");
                    ScrollArea::vertical().max_height(400.0).show(ui, |ui| {
                        joints_in_chain.iter().for_each(|joint_in_chain| {
                            let joint = &urdf_module.joints[joint_in_chain.joint_idx];
                            let dof_idxs = &dof_module.joint_idx_to_dof_idxs_mapping[joint_in_chain.joint_idx];
                            let parent_link_idx = joint_in_chain.parent_link_idx;
                            let child_link_idx = joint_in_chain.child_link_idx;

                            let mut hovered = false;
                            ui.label(format!("Joint {}: {}", joint_in_chain.joint_idx, joint.name));
                            ui.label(format!("Joint type {:?}, num dofs: {}", joint.joint_type, dof_idxs.len()));
                            for dof_idx in dof_idxs {
                                let bounds = bounds_module.bounds[*dof_idx];
                                let resp = ui.add(Slider::new(&mut robot_state[*dof_idx], bounds.0..=bounds.1));
                                if resp.hovered() { hovered = true; }
                                ui.horizontal(|ui| {
                                    let resp = ui.button("0.0");
                                    if resp.hovered() || resp.contains_pointer() { hovered = true; }
                                    if resp.clicked() { robot_state[*dof_idx] = 0.0; }

                                    let resp = ui.button("+0.1");
                                    if resp.hovered() || resp.contains_pointer() { hovered = true; }
                                    if resp.clicked() { robot_state[*dof_idx] += 0.1; }

                                    let resp = ui.button("-0.1");
                                    if resp.hovered() || resp.contains_pointer() { hovered = true; }
                                    if resp.clicked() { robot_state[*dof_idx] -= 0.1; }
                                });

                                if hovered {
                                    color_change_engine.add_momentary_request(ColorChangeRequest::new(ColorChangeRequestType::medium_priority_color(0.0, 0.2, 1.0, 0.8), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx), ChainMeshComponent::LinkIdx(parent_link_idx)])));

                                    color_change_engine.add_momentary_request(ColorChangeRequest::new(ColorChangeRequestType::medium_priority_color(0.2, 1.0, 0.0, 0.8), Signature::new_chain_link_mesh(vec![ChainMeshComponent::LinkIdx(child_link_idx), ChainMeshComponent::ChainInstanceIdx(chain_instance_idx)])));
                                }
                            }
                            ui.separator();
                        });
                    });

                    ui.separator();

                    if ui.button("Reset").clicked() {
                        robot_state.iter_mut().for_each(|x| *x = 0.0);
                    }
                });
            }
        });
    }

    pub fn action_chain_sliders_egui(&self, color_change_engine: &mut ResMut<ColorChangeEngine>, query: &mut Query<&mut ChainState>, ui: &mut Ui) {
        Self::action_chain_sliders_egui_static(self.chain_instance_idx, &self.urdf_module, &self.chain_module, &self.dof_module, &self.bounds_module, color_change_engine, query, ui);
    }

    pub fn get_system_side_panel_left<F: FnMut(&mut Ui, &BevyChainSlidersEguiRaw) + 'static>(self, mut f: F) -> impl FnMut(EguiContexts, Query<&mut ChainState>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>, ResMut<ColorChangeEngine>) + 'static {
        return move |mut egui_contexts: EguiContexts, mut query: Query<&mut ChainState>, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window1, With<PrimaryWindow>>, mut color_change_engine: ResMut<ColorChangeEngine>| {
            let self_clone = self.clone();
            SidePanel::left(format!("chain_sliders_side_panel_chain_instance_idx_{}", self.chain_instance_idx))
                .show(egui_contexts.ctx_mut(),  |ui| {
                    self_clone.action_chain_sliders_egui(&mut color_change_engine, &mut query, ui);
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
    pub fn action_chain_sliders_egui(&self, color_change_engine: &mut ResMut<ColorChangeEngine>, query: &mut Query<&mut ChainState>, ui: &mut Ui) {
        BevyChainSlidersEguiRaw::action_chain_sliders_egui_static(self.chain_instance_idx, &self.chain.urdf_module, &self.chain.chain_module, &self.chain.dof_module, &self.chain.bounds_module, color_change_engine, query, ui);
    }
    pub fn get_system_side_panel_left<F: FnMut(&mut Ui, &BevyChainSlidersEguiRaw) + 'static>(self, f: F) -> impl FnMut(EguiContexts, Query<&mut ChainState>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>, ResMut<ColorChangeEngine>) + 'static {
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
pub struct BevyChainLinkVisibilitySelectorRaw {
    pub chain_instance_idx: usize,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    plain_meshes_glb_visibility: Vec<bool>,
    plain_meshes_obj_visibility: Vec<bool>,
    convex_hull_meshes_obj_visibility: Vec<bool>,
    convex_decomposition_meshes_obj_visibility: Vec<bool>,
    bounding_spheres_full_visibility: Vec<bool>,
    obbs_full_visibility: Vec<bool>,
    bounding_spheres_decomposition_visibility: Vec<bool>,
    obbs_decomposition_visibility: Vec<bool>
}
impl BevyChainLinkVisibilitySelectorRaw {
    pub fn new(chain_instance_idx: usize, urdf_module: ApolloURDFNalgebraModule, chain_module: ApolloChainModule) -> Self {
        let num_links = urdf_module.links.len();

        let plain_meshes_glb_visibility = vec![false; num_links];
        let plain_meshes_obj_visibility = vec![true; num_links];
        let convex_hull_meshes_obj_visibility = vec![false; num_links];
        let convex_decomposition_meshes_obj_visibility = vec![false; num_links];
        let bounding_spheres_full_visibility = vec![false; num_links];
        let obbs_full_visibility = vec![false; num_links];
        let bounding_spheres_decomposition_visibility = vec![false; num_links];
        let obbs_decomposition_visibility = vec![false; num_links];

        Self {
            chain_instance_idx,
            urdf_module,
            chain_module,
            plain_meshes_glb_visibility,
            plain_meshes_obj_visibility,
            convex_hull_meshes_obj_visibility,
            convex_decomposition_meshes_obj_visibility,
            bounding_spheres_full_visibility,
            obbs_full_visibility,
            bounding_spheres_decomposition_visibility,
            obbs_decomposition_visibility,
        }
    }
    pub fn action_chain_link_visibility_selector_static(chain_instance_idx: usize,
                                                        urdf_module: &ApolloURDFNalgebraModule,
                                                        chain_module: &ApolloChainModule,
                                                        plain_meshes_glb_visibility: &mut Vec<bool>,
                                                        plain_meshes_obj_visibility: &mut Vec<bool>,
                                                        convex_hull_meshes_obj_visibility: &mut Vec<bool>,
                                                        convex_decomposition_meshes_obj_visibility: &mut Vec<bool>,
                                                        bounding_spheres_full_visibility: &mut Vec<bool>,
                                                        obbs_full_visibility: &mut Vec<bool>,
                                                        bounding_spheres_decomposition_visibility: &mut Vec<bool>,
                                                        obbs_decomposition_visibility: &mut Vec<bool>,
                                                        ui: &mut Ui,
                                                        visibility_change_engine: &mut ResMut<VisibilityChangeEngine>) {
        ui.heading("Link Visibility");
        ui.group(|ui| {
            ScrollArea::vertical().id_source("sa1").max_height(400.0).show(ui, |ui| {
                for (link, link_in_chain) in urdf_module.links.iter().zip(&chain_module.links_in_chain) {
                    let link_idx = link_in_chain.link_idx;
                    let link_name = link.name.clone();

                    ui.label(format!("link {}, {}", link_idx, link_name));
                    ui.checkbox(&mut plain_meshes_obj_visibility[link_idx], "Plain mesh obj");
                    ui.checkbox(&mut plain_meshes_glb_visibility[link_idx], "Plain mesh glb");
                    ui.checkbox(&mut convex_hull_meshes_obj_visibility[link_idx], "Convex shape");
                    ui.checkbox(&mut convex_decomposition_meshes_obj_visibility[link_idx], "Convex decomp.");
                    ui.checkbox(&mut bounding_spheres_full_visibility[link_idx], "Bounding sphere full");
                    ui.checkbox(&mut obbs_full_visibility[link_idx], "OBB full");
                    ui.checkbox(&mut bounding_spheres_decomposition_visibility[link_idx], "Bounding sphere decomp.");
                    ui.checkbox(&mut obbs_decomposition_visibility[link_idx], "OBB decomp.");

                    ui.separator();

                    let b = plain_meshes_obj_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::Plain),
                                                                                                                                                                                     ChainMeshComponent::MeshType(MeshType::OBJ)])));

                    let b = plain_meshes_glb_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::Plain),
                                                                                                                                                                                     ChainMeshComponent::MeshType(MeshType::GLB)])));

                    let b = convex_hull_meshes_obj_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::ConvexHull),
                                                                                                                                                                                     ChainMeshComponent::MeshType(MeshType::OBJ)])));
                    let b = convex_decomposition_meshes_obj_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::ConvexDecomposition),
                                                                                                                                                                                     ChainMeshComponent::MeshType(MeshType::OBJ)])));
                    let b = bounding_spheres_full_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx), ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::BoundingSphereFull)])));

                    let b = obbs_full_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::OBBFull)])));
                    let b = bounding_spheres_decomposition_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::BoundingSphereDecomposition)])));
                    let b = obbs_decomposition_visibility[link_idx];
                    visibility_change_engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::new_from_bool(b), Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainInstanceIdx(chain_instance_idx),
                                                                                                                                                                                     ChainMeshComponent::LinkIdx(link_idx),
                                                                                                                                                                                     ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::OBBDecomposition)])));
                }
            });
        });

        ui.separator();

        ui.group(|ui| {
            ScrollArea::vertical().id_source("sa2").max_height(400.0).show(ui, |ui| {
                if ui.button("Select all plain meshes obj").clicked() {
                    plain_meshes_obj_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all plain meshes obj").clicked() {
                    plain_meshes_obj_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all plain meshes glb").clicked() {
                    plain_meshes_glb_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all plain meshes glb").clicked() {
                    plain_meshes_glb_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all convex hulls").clicked() {
                    convex_hull_meshes_obj_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all convex hulls").clicked() {
                    convex_hull_meshes_obj_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all convex decomp.").clicked() {
                    convex_decomposition_meshes_obj_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all convex decomp").clicked() {
                    convex_decomposition_meshes_obj_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all bounding spheres").clicked() {
                    bounding_spheres_full_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all bounding spheres").clicked() {
                    bounding_spheres_full_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all obbs").clicked() {
                    obbs_full_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all obbs").clicked() {
                    obbs_full_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all bounding sphere decomps.").clicked() {
                    bounding_spheres_decomposition_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all bounding sphere decomps.").clicked() {
                    bounding_spheres_decomposition_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();

                if ui.button("Select all obbs decomps.").clicked() {
                    obbs_decomposition_visibility.iter_mut().for_each(|x| *x = true);
                }

                if ui.button("Deselect all obbs decomps.").clicked() {
                    obbs_decomposition_visibility.iter_mut().for_each(|x| *x = false);
                }

                ui.separator();
            });
        });
    }

    pub fn action_chain_link_visibility_selector(&mut self,
                                                 ui: &mut Ui,
                                                 visibility_change_engine: &mut ResMut<VisibilityChangeEngine>) {
        Self::action_chain_link_visibility_selector_static(self.chain_instance_idx, &self.urdf_module, &self.chain_module, &mut self.plain_meshes_glb_visibility, &mut self.plain_meshes_obj_visibility, &mut self.convex_hull_meshes_obj_visibility, &mut self.convex_decomposition_meshes_obj_visibility, &mut self.bounding_spheres_full_visibility, &mut self.obbs_full_visibility, &mut self.bounding_spheres_decomposition_visibility, &mut self.obbs_decomposition_visibility, ui, visibility_change_engine);
    }

    pub fn get_system_side_panel_left(self) -> impl FnMut(EguiContexts, ResMut<VisibilityChangeEngine>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>) + 'static {
        let mut self_clone = self.clone();
        move |mut egui_contexts: EguiContexts, mut visibility_change_engine: ResMut<VisibilityChangeEngine>, mut cursor_is_over_eugi: ResMut<CursorIsOverEgui>, query: Query<&Window1, With<PrimaryWindow>>| {
            SidePanel::left("chain_link_visibility_selctor_side_panel").show(egui_contexts.ctx_mut(), |ui| {
                self_clone.action_chain_link_visibility_selector(ui, &mut visibility_change_engine);
                set_cursor_is_over_egui_default(ui, &mut cursor_is_over_eugi, &query);
            });
        }
    }
}

#[derive(Clone)]
pub struct BevyChainLinkVisibilitySelector {
    pub chain_instance_idx: usize,
    pub chain: Arc<ChainNalgebra>
}
impl BevyChainLinkVisibilitySelector {
    pub fn get_system_side_panel_left(self) -> impl FnMut(EguiContexts, ResMut<VisibilityChangeEngine>, ResMut<CursorIsOverEgui>, Query<&Window1, With<PrimaryWindow>>) + 'static {
        let c = BevyChainLinkVisibilitySelectorRaw::new(self.chain_instance_idx, self.chain.urdf_module.clone(), self.chain.chain_module.clone());
        c.get_system_side_panel_left()
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
    pub fn action_pose_chain(&self, state: &V, global_offset: &ISE3q, query: &mut Query<(&mut Transform, &ChainLinkMesh), Without<LinkApproximatingShape>>) {
        let fk_res = RobotKinematicsFunctions::fk(state, &self.urdf_module, &self.chain_module, &self.dof_module);
        query.iter_mut().for_each(|(mut x, y)| {
            if y.chain_instance_idx == self.chain_instance_idx {
                let link_idx = y.link_idx;
                *x = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&global_offset.group_operator(&fk_res[link_idx]));
            }
        });
    }
    pub fn action_pose_chain_for_approximating_shapes(&self, state: &V, global_offset: &ISE3q, query: &mut Query<(&mut Transform, &ChainLinkMesh, &OffsetFrame)>) {
        let fk_res = RobotKinematicsFunctions::fk(state, &self.urdf_module, &self.chain_module, &self.dof_module);
        query.iter_mut().for_each(|(mut x, y, z)| {
            if y.chain_instance_idx == self.chain_instance_idx {
                let link_idx = y.link_idx;
                let t = global_offset.group_operator(&fk_res[link_idx]).group_operator(&z.0);
                *x = TransformUtils::util_convert_pose_to_y_up_bevy_transform(&t);
            }
        });
    }
    pub fn get_system1(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh), Without<LinkApproximatingShape>>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        return move | mut query1: Query<(&mut Transform, &ChainLinkMesh), Without<LinkApproximatingShape>>, query2: Query<&mut ChainState, Changed<ChainState>>| {
            for x in query2.iter() {
                if x.chain_instance_idx == self.chain_instance_idx {
                    self.action_pose_chain(&x.state, &x.global_offset, &mut query1);
                }
            }
        }
    }
    pub fn get_system2(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh, &OffsetFrame)>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        return move | mut query1: Query<(&mut Transform, &ChainLinkMesh, &OffsetFrame)>, query2: Query<&mut ChainState, Changed<ChainState>>| {
            for x in query2.iter() {
                if x.chain_instance_idx == self.chain_instance_idx {
                    self.action_pose_chain_for_approximating_shapes(&x.state, &x.global_offset, &mut query1);
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
    pub fn get_system(self) -> impl FnMut(Query<(&mut Transform, &ChainLinkMesh), Without<LinkApproximatingShape>>, Query<&mut ChainState, Changed<ChainState>>) + 'static {
        let raw = BevyChainStateUpdaterLoopRaw {
            chain_instance_idx: self.chain_instance_idx,
            urdf_module: self.chain.urdf_module.clone(),
            chain_module: self.chain.chain_module.clone(),
            dof_module: self.chain.dof_module.clone(),
        };
        return raw.get_system1()
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Component)]
pub struct ChainState {
    pub chain_instance_idx: usize,
    pub state: V,
    pub global_offset: ISE3q
}

#[derive(Clone, Debug, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ChainMeshesRepresentation {
    BoundingSphereFull,
    OBBFull,
    BoundingSphereDecomposition,
    OBBDecomposition,
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

#[derive(Clone, Debug, Component)]
pub struct LinkApproximatingShape;

#[derive(Clone, Debug, Component)]
pub struct OffsetFrame(pub ISE3q);
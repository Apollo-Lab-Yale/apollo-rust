pub mod apollo_bevy_utils;

use crate::apollo_bevy_utils::camera::CameraSystems;
use crate::apollo_bevy_utils::chain::{
    BevyChainLinkVisibilitySelector, BevyChainMotionPlayback, BevyChainProximityVisualizer,
    BevyChainSlidersEgui, BevyChainStateUpdaterLoopRaw, BevySpawnChainLinkApproximationsRaw,
    BevySpawnChainMeshesRaw, ChainMeshesRepresentation, ChainState,
};
use crate::apollo_bevy_utils::colors::{ColorChangeEngine, ColorChangeSystems};
use crate::apollo_bevy_utils::egui::{
    reset_cursor_is_over_egui, set_cursor_is_over_egui_default, CursorIsOverEgui,
};
use crate::apollo_bevy_utils::meshes::MeshType;
use crate::apollo_bevy_utils::signatures::{ChainMeshComponent, Signature};
use crate::apollo_bevy_utils::transform::TransformUtils;
use crate::apollo_bevy_utils::transform_gizmo::{TransformGizmoEngine, TransformGizmoSystems};
use crate::apollo_bevy_utils::viewport_visuals::ViewportVisualsActions;
use crate::apollo_bevy_utils::visibility::{
    BaseVisibility, VisibilityChangeEngine, VisibilityChangeRequest, VisibilityChangeRequestType,
    VisibilityChangeSystems,
};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_interpolation::InterpolatorTraitLite;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::rotation_matrices::ApolloRotation3Trait;
use apollo_rust_spatial::translations::ApolloTranslation3;
use bevy::log::LogPlugin;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{ComboBox, SidePanel};
use bevy_egui::{EguiContexts, EguiPlugin};
use bevy_mod_outline::{
    AsyncSceneInheritOutlinePlugin, AutoGenerateOutlineNormalsPlugin, OutlinePlugin,
};
use bevy_obj::ObjPlugin;
use std::path::PathBuf;
use std::sync::Arc;
use transform_gizmo_bevy::{
    EnumSet, GizmoMode, GizmoOptions, GizmoOrientation, GizmoVisuals, TransformGizmoPlugin,
};

pub trait ApolloBevyTrait {
    fn apollo_bevy_base(self, disable_log: bool, headless: bool) -> Self;
    fn apollo_bevy_robotics_base(self, disable_log: bool, headless: bool) -> Self;
    fn apollo_bevy_pan_orbit_camera(self) -> Self;
    fn apollo_bevy_pan_orbit_three_style_camera(self) -> Self;
    fn apollo_bevy_starter_lights(self) -> Self;
    fn apollo_bevy_robotics_scene_visuals_start(self) -> Self;
    fn apollo_bevy_spawn_transform_gizmos(self, initial_transforms: Vec<Option<ISE3q>>) -> Self;
    fn apollo_bevy_spawn_chain(
        self,
        chain: &ChainNalgebra,
        chain_instance_idx: usize,
        global_offset: ISE3q,
        mesh_specs: Vec<(ChainMeshesRepresentation, MeshType, BaseVisibility)>,
        path_to_bevy_assets: &PathBuf,
    ) -> Self;
    fn apollo_bevy_spawn_chain_default(
        self,
        chain: &ChainNalgebra,
        chain_instance_idx: usize,
        global_offset: ISE3q,
    ) -> Self;
    fn apollo_bevy_spawn_chain_raw(
        self,
        resources_sub_directory: &ResourcesSubDirectory,
        urdf_module: &ApolloURDFNalgebraModule,
        chain_module: &ApolloChainModule,
        dof_module: &ApolloDOFModule,
        plain_meshes_module: &ApolloPlainMeshesModule,
        convex_hull_meshes_module: &ApolloConvexHullMeshesModule,
        convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule,
        link_shape_approximations_module: &ApolloLinkShapesApproximationsModule,
        chain_instance_idx: usize,
        global_offset: ISE3q,
        mesh_specs: Vec<(ChainMeshesRepresentation, MeshType, BaseVisibility)>,
        path_to_bevy_assets: &PathBuf,
    ) -> Self;
    fn apollo_bevy_draw_frame(self, frame: &ISE3q, scale: f64) -> Self;
    fn apollo_bevy_chain_display(self, chain: &ChainNalgebra) -> Self;
    fn apollo_bevy_chain_proximity_vis(self, chain: &ChainNalgebra, include_sliders: bool) -> Self;
    fn apollo_bevy_chain_motion_playback<I: InterpolatorTraitLite + 'static>(
        self,
        chain_instance_idx: usize,
        interpolator: I,
    ) -> Self;
}
impl ApolloBevyTrait for App {
    fn apollo_bevy_base(self, disable_log: bool, headless: bool) -> Self {
        let mut out = App::from(self);

        let mut plugins = DefaultPlugins.build();

        if disable_log {
            plugins = plugins.disable::<LogPlugin>();
        }

        if headless {
            plugins = plugins
                .set(WindowPlugin {
                    primary_window: None,
                    ..Default::default()
                })
                .disable::<bevy::winit::WinitPlugin>();
        } else {
            plugins = plugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "APOLLO TOOLBOX".to_string(),
                    ..Default::default()
                }),
                ..Default::default()
            });
        }

        out.add_plugins(plugins);

        let mut gizmo_modes = EnumSet::new();
        gizmo_modes = gizmo_modes | GizmoMode::all_translate();
        gizmo_modes = gizmo_modes | GizmoMode::all_rotate();

        out.insert_resource(ClearColor(Color::srgb(0.9, 0.9, 0.92)))
            .insert_resource(Msaa::default())
            .add_plugins(EguiPlugin)
            .add_plugins(ObjPlugin)
            .add_plugins((
                OutlinePlugin,
                AutoGenerateOutlineNormalsPlugin,
                AsyncSceneInheritOutlinePlugin,
            ))
            .add_plugins(TransformGizmoPlugin)
            .insert_resource(GizmoOptions {
                visuals: GizmoVisuals {
                    gizmo_size: 60.0,
                    ..Default::default()
                },
                gizmo_modes,
                group_targets: false,
                ..Default::default()
            })
            .insert_resource(CursorIsOverEgui(false))
            .insert_resource(VisibilityChangeEngine::new())
            .insert_resource(TransformGizmoEngine::new())
            .add_systems(
                Last,
                VisibilityChangeSystems::system_reset_base_visibilities
                    .before(VisibilityChangeSystems::system_set_base_visibility_request_changes)
                    .before(
                        VisibilityChangeSystems::system_set_momentary_visibility_request_changes,
                    ),
            )
            .add_systems(
                Last,
                VisibilityChangeSystems::system_set_base_visibility_request_changes,
            )
            .add_systems(
                Last,
                VisibilityChangeSystems::system_set_momentary_visibility_request_changes,
            )
            .add_systems(
                Last,
                VisibilityChangeSystems::system_clear_visibility_request_changes
                    .after(VisibilityChangeSystems::system_set_base_visibility_request_changes)
                    .after(
                        VisibilityChangeSystems::system_set_momentary_visibility_request_changes,
                    ),
            )
            .insert_resource(ColorChangeEngine::new())
            .add_systems(
                Last,
                ColorChangeSystems::system_reset_base_colors
                    .before(ColorChangeSystems::system_set_base_color_request_changes)
                    .before(ColorChangeSystems::system_set_momentary_color_request_changes),
            )
            .add_systems(
                Last,
                ColorChangeSystems::system_set_base_color_request_changes,
            )
            .add_systems(
                Last,
                ColorChangeSystems::system_set_momentary_color_request_changes,
            )
            .add_systems(
                Last,
                ColorChangeSystems::system_clear_color_request_changes
                    .after(ColorChangeSystems::system_set_base_color_request_changes)
                    .after(ColorChangeSystems::system_set_momentary_color_request_changes),
            )
            .add_systems(Last, reset_cursor_is_over_egui)
            .add_systems(
                Last,
                TransformGizmoSystems::system_transform_gizmos_set_transforms_for_get,
            )
            .add_systems(
                Update,
                TransformGizmoSystems::system_transform_gizmos_set_transforms,
            )
            .add_systems(
                PreUpdate,
                TransformGizmoSystems::system_transform_gizmos_inserts,
            )
            .add_systems(
                First,
                TransformGizmoSystems::system_transform_gizmos_deletes,
            )
            .add_systems(
                Update,
                |keys: Res<ButtonInput<KeyCode>>, mut options: ResMut<GizmoOptions>| {
                    if keys.just_pressed(KeyCode::Tab) {
                        match &options.gizmo_orientation {
                            GizmoOrientation::Global => {
                                options.gizmo_orientation = GizmoOrientation::Local
                            }
                            GizmoOrientation::Local => {
                                options.gizmo_orientation = GizmoOrientation::Global
                            }
                        }
                    }
                },
            );

        out
    }

    fn apollo_bevy_robotics_base(self, disable_log: bool, headless: bool) -> Self {
        let out = App::new()
            .apollo_bevy_base(disable_log, headless)
            .apollo_bevy_pan_orbit_three_style_camera()
            .apollo_bevy_robotics_scene_visuals_start()
            .apollo_bevy_starter_lights();

        return out;
    }

    fn apollo_bevy_pan_orbit_camera(self) -> Self {
        let mut out = App::from(self);

        out.add_systems(Startup, CameraSystems::system_spawn_pan_orbit_camera)
            .add_systems(PostUpdate, CameraSystems::system_pan_orbit_camera);

        out
    }

    fn apollo_bevy_pan_orbit_three_style_camera(self) -> Self {
        let mut out = App::from(self);

        out.add_systems(
            Startup,
            CameraSystems::system_spawn_pan_orbit_three_style_camera,
        )
        .add_systems(
            PostUpdate,
            CameraSystems::system_pan_orbit_three_style_camera,
        );

        out
    }

    fn apollo_bevy_starter_lights(self) -> Self {
        let mut out = App::from(self);

        out.add_systems(Startup, |mut commands: Commands| {
            commands.spawn(PointLightBundle {
                point_light: PointLight { ..default() },
                transform: Transform::from_xyz(4.0, 4.0, 4.0),
                ..default()
            });
            commands.spawn(PointLightBundle {
                point_light: PointLight { ..default() },
                transform: Transform::from_xyz(1.0, 2.0, -4.0),
                ..default()
            });
        });

        out
    }

    fn apollo_bevy_robotics_scene_visuals_start(self) -> Self {
        let mut out = App::from(self);

        out.add_systems(
            Startup,
            |mut commands: Commands,
             mut meshes: ResMut<Assets<Mesh>>,
             mut materials: ResMut<Assets<StandardMaterial>>| {
                ViewportVisualsActions::action_draw_robotics_grid(
                    &mut commands,
                    &mut meshes,
                    &mut materials,
                );
            },
        );

        out
    }

    fn apollo_bevy_spawn_transform_gizmos(self, initial_transforms: Vec<Option<ISE3q>>) -> Self {
        let mut out = App::from(self);

        out.add_systems(
            Startup,
            move |mut transform_gizmo_engine: ResMut<TransformGizmoEngine>| {
                initial_transforms.iter().for_each(|x| {
                    transform_gizmo_engine.insert_new_transform_gizmo(x.clone());
                });
            },
        );

        out
    }

    fn apollo_bevy_spawn_chain(
        self,
        chain: &ChainNalgebra,
        chain_instance_idx: usize,
        global_offset: ISE3q,
        mesh_specs: Vec<(ChainMeshesRepresentation, MeshType, BaseVisibility)>,
        path_to_bevy_assets: &PathBuf,
    ) -> Self {
        return self.apollo_bevy_spawn_chain_raw(
            &chain.resources_sub_directory(),
            &chain.urdf_module,
            &chain.chain_module,
            &chain.dof_module,
            &chain.plain_meshes_module,
            &chain.convex_hull_meshes_module,
            &chain.convex_decomposition_meshes_module,
            &chain.link_shapes_approximations_module,
            chain_instance_idx,
            global_offset,
            mesh_specs,
            path_to_bevy_assets,
        );
    }

    fn apollo_bevy_spawn_chain_default(
        self,
        chain: &ChainNalgebra,
        chain_instance_idx: usize,
        global_offset: ISE3q,
    ) -> Self {
        self.apollo_bevy_spawn_chain(
            chain,
            chain_instance_idx,
            global_offset,
            get_default_mesh_specs(),
            &PathBuf::new_from_default_apollo_bevy_assets_dir(),
        )
    }

    fn apollo_bevy_spawn_chain_raw(
        self,
        resources_sub_directory: &ResourcesSubDirectory,
        urdf_module: &ApolloURDFNalgebraModule,
        chain_module: &ApolloChainModule,
        dof_module: &ApolloDOFModule,
        plain_meshes_module: &ApolloPlainMeshesModule,
        convex_hull_meshes_module: &ApolloConvexHullMeshesModule,
        convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule,
        link_shape_approximations_module: &ApolloLinkShapesApproximationsModule,
        chain_instance_idx: usize,
        global_offset: ISE3q,
        mesh_specs: Vec<(ChainMeshesRepresentation, MeshType, BaseVisibility)>,
        path_to_bevy_assets: &PathBuf,
    ) -> Self {
        let mut out = App::from(self);

        let zero_state = V::new(&vec![0.0; dof_module.num_dofs]);
        let global_offset_clone = global_offset.clone();

        out.world_mut().spawn(ChainState {
            chain_instance_idx,
            state: zero_state.clone(),
            global_offset: global_offset_clone,
        });

        mesh_specs.iter().for_each(|(x, y, z)| {
            let c = BevySpawnChainMeshesRaw {
                chain_instance_idx,
                chain_meshes_representation: x.clone(),
                mesh_type: y.clone(),
                resources_sub_directory: resources_sub_directory.clone(),
                urdf_module: urdf_module.clone(),
                chain_module: chain_module.clone(),
                dof_module: dof_module.clone(),
                plain_meshes_module: plain_meshes_module.clone(),
                convex_hull_meshes_module: convex_hull_meshes_module.clone(),
                convex_decomposition_meshes_module: convex_decomposition_meshes_module.clone(),
                link_shapes_approximations_module: link_shape_approximations_module.clone(),
                path_to_bevy_assets: path_to_bevy_assets.clone(),
                state: zero_state.clone(),
                base_visibility_mode: z.clone(),
            };
            out.add_systems(Startup, c.get_system());
        });

        let c = BevySpawnChainLinkApproximationsRaw {
            chain_instance_idx,
            resources_sub_directory: resources_sub_directory.clone(),
            urdf_module: urdf_module.clone(),
            chain_module: chain_module.clone(),
            dof_module: dof_module.clone(),
            link_shapes_approximations_module: link_shape_approximations_module.clone(),
            state: zero_state.clone(),
            base_visibility_mode: BaseVisibility::Off,
        };
        out.add_systems(Startup, c.get_system());

        let c = BevyChainStateUpdaterLoopRaw {
            chain_instance_idx,
            urdf_module: urdf_module.clone(),
            chain_module: chain_module.clone(),
            dof_module: dof_module.clone(),
        };
        out.add_systems(PostUpdate, c.clone().get_system1());
        out.add_systems(PostUpdate, c.get_system2());

        out
    }

    fn apollo_bevy_draw_frame(self, frame: &ISE3q, scale: f64) -> Self {
        let mut out = App::from(self);

        let position = frame.0.translation.to_vector3();
        let frame_vectors = frame.0.rotation.to_rotation_matrix().frame_vectors();

        let p1 =
            TransformUtils::util_convert_z_up_v3_to_y_up_vec3(position + scale * frame_vectors[0]);
        let p2 =
            TransformUtils::util_convert_z_up_v3_to_y_up_vec3(position + scale * frame_vectors[1]);
        let p3 =
            TransformUtils::util_convert_z_up_v3_to_y_up_vec3(position + scale * frame_vectors[2]);

        let center = TransformUtils::util_convert_z_up_v3_to_y_up_vec3(position);

        out.add_systems(Update, move |mut gizmos: Gizmos| {
            gizmos.line(center, p1, Color::srgb(1.0, 0.0, 0.0));
            gizmos.line(center, p2, Color::srgb(0.0, 1.0, 0.0));
            gizmos.line(center, p3, Color::srgb(0.0, 0.0, 1.0));
        });

        out
    }

    fn apollo_bevy_chain_display(self, chain: &ChainNalgebra) -> Self {
        let mut out = App::from(self);

        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;

        let arc_robot = Arc::new(chain.clone());
        let c = BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: arc_robot.clone(),
            color_changes: true,
        };
        out.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

        let c = BevyChainLinkVisibilitySelector {
            chain_instance_idx: 0,
            chain: arc_robot.clone(),
        };
        out.add_systems(Update, c.get_system_side_panel_left().after(S1));

        out
    }

    fn apollo_bevy_chain_proximity_vis(self, chain: &ChainNalgebra, include_sliders: bool) -> Self {
        let mut app = App::from(self);

        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;

        let arc_robot = Arc::new(chain.clone());

        if include_sliders {
            let c = BevyChainSlidersEgui {
                chain_instance_idx: 0,
                chain: arc_robot.clone(),
                color_changes: false,
            };
            app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));
        }

        let mut link_shape_mode_a = LinkShapeMode::Full;
        let mut link_shape_rep_a = LinkShapeRep::ConvexHull;
        let mut selected_idxs = None;

        let c = BevyChainProximityVisualizer {
            chain_instance_idx_a: 0,
            chain_a: arc_robot.clone(),
            chain_instance_idx_b: 0,
            chain_b: arc_robot.clone(),
        }
        .to_raw();

        let a = arc_robot.clone();
        app.add_systems(
            Update,
            (move |mut egui_contexts: EguiContexts,
                   mut color_change_engine: ResMut<ColorChangeEngine>,
                   mut visibility_change_engine: ResMut<VisibilityChangeEngine>,
                   query: Query<&ChainState>,
                   mut cursor_is_over_egui: ResMut<CursorIsOverEgui>,
                   query2: Query<&Window, With<PrimaryWindow>>,
                   mut gizmos: Gizmos| {
                let chain_state_a = query
                    .iter()
                    .find(|x| x.chain_instance_idx == 0)
                    .expect("error");

                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::Off,
                    Signature::new_chain_link_mesh(vec![]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::from_link_shape_mode_and_link_shape_rep(
                                &link_shape_mode_a,
                                &link_shape_rep_a,
                            ),
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::Plain,
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));

                let stats = a
                    .link_shapes_distance_statistics_module
                    .get_stats(&link_shape_rep_a, &link_shape_mode_a);
                let skips = a
                    .link_shapes_skips_nalgebra_module
                    .get_skips(link_shape_mode_a, link_shape_rep_a);
                SidePanel::left("proximity_visualizer").show(egui_contexts.ctx_mut(), |ui| {
                    ui.heading("Pairwise Distances");
                    c.action_chain_proximity_visualizer(
                        ui,
                        &chain_state_a.state,
                        &chain_state_a.state,
                        &link_shape_mode_a,
                        &link_shape_mode_a,
                        &link_shape_rep_a,
                        &link_shape_rep_a,
                        Some(skips),
                        Some(&stats.averages),
                        &mut selected_idxs,
                        &mut color_change_engine,
                        &mut gizmos,
                    );

                    ui.separator();

                    ComboBox::from_label("Link shape mode")
                        .selected_text(format!("{:?}", link_shape_mode_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Full,
                                "Full",
                            );
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Decomposition,
                                "Decomposition",
                            );
                        });
                    ComboBox::from_label("Link shape rep")
                        .selected_text(format!("{:?}", link_shape_rep_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::ConvexHull,
                                "Convex Hull",
                            );
                            ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::OBB, "OBB");
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::BoundingSphere,
                                "Bounding Sphere",
                            );
                        });

                    ui.separator();

                    if ui.button("Deselect").clicked() {
                        selected_idxs = None;
                    }

                    set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
                });
            })
            .after(S1),
        );

        app
    }

    fn apollo_bevy_chain_motion_playback<I: InterpolatorTraitLite + 'static>(
        self,
        chain_instance_idx: usize,
        interpolator: I,
    ) -> Self {
        let mut app = App::from(self);

        let c = BevyChainMotionPlayback { chain_instance_idx };
        app.add_systems(Update, c.get_system_bottom_panel(Arc::new(interpolator)));

        app
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

pub trait ApolloChainBevyTrait {
    fn bevy_display(&self) {
        self.bevy_display_app().run();
    }

    fn bevy_display_app(&self) -> App;

    fn bevy_proximity_vis(&self) {
        self.bevy_proximity_vis_app().run();
    }

    fn bevy_proximity_vis_app(&self) -> App;

    fn bevy_display_with_other_chain(&self, other: &ChainNalgebra) {
        self.bevy_display_with_other_chain_app(other).run();
    }

    fn bevy_display_with_other_chain_app(&self, other: &ChainNalgebra) -> App;

    fn bevy_proximity_vis_with_other_chain(&self, other: &ChainNalgebra) {
        self.bevy_proximity_vis_with_other_chain_app(other).run();
    }

    fn bevy_proximity_vis_with_other_chain_app(&self, other: &ChainNalgebra) -> App;

    fn bevy_motion_playback_app<I: InterpolatorTraitLite + 'static>(&self, interpolator: I) -> App;

    fn bevy_motion_playback<I: InterpolatorTraitLite + 'static>(&self, interpolator: I) {
        self.bevy_motion_playback_app(interpolator).run();
    }
}
impl ApolloChainBevyTrait for ChainNalgebra {
    fn bevy_display_app(&self) -> App {
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;

        let mut app = App::new()
            .apollo_bevy_robotics_base(true, false)
            .apollo_bevy_spawn_chain(
                self,
                0,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            );

        let arc_robot = Arc::new(self.clone());
        let c = BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: arc_robot.clone(),
            color_changes: true,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

        let c = BevyChainLinkVisibilitySelector {
            chain_instance_idx: 0,
            chain: arc_robot.clone(),
        };
        app.add_systems(Update, c.get_system_side_panel_left().after(S1));

        app
    }

    fn bevy_proximity_vis_app(&self) -> App {
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;

        let mut app = App::new()
            .apollo_bevy_robotics_base(true, false)
            .apollo_bevy_spawn_chain(
                self,
                0,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            );

        let arc_robot = Arc::new(self.clone());
        let c = BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: arc_robot.clone(),
            color_changes: false,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

        let mut link_shape_mode_a = LinkShapeMode::Full;
        let mut link_shape_rep_a = LinkShapeRep::ConvexHull;
        let mut selected_idxs = None;

        let c = BevyChainProximityVisualizer {
            chain_instance_idx_a: 0,
            chain_a: arc_robot.clone(),
            chain_instance_idx_b: 0,
            chain_b: arc_robot.clone(),
        }
        .to_raw();

        let a = arc_robot.clone();
        app.add_systems(
            Update,
            (move |mut egui_contexts: EguiContexts,
                   mut color_change_engine: ResMut<ColorChangeEngine>,
                   mut visibility_change_engine: ResMut<VisibilityChangeEngine>,
                   query: Query<&ChainState>,
                   mut cursor_is_over_egui: ResMut<CursorIsOverEgui>,
                   query2: Query<&Window, With<PrimaryWindow>>,
                   mut gizmos: Gizmos| {
                let chain_state_a = query
                    .iter()
                    .find(|x| x.chain_instance_idx == 0)
                    .expect("error");

                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::Off,
                    Signature::new_chain_link_mesh(vec![]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::from_link_shape_mode_and_link_shape_rep(
                                &link_shape_mode_a,
                                &link_shape_rep_a,
                            ),
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::Plain,
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));

                let stats = a
                    .link_shapes_distance_statistics_module
                    .get_stats(&link_shape_rep_a, &link_shape_mode_a);
                let skips = a
                    .link_shapes_skips_nalgebra_module
                    .get_skips(link_shape_mode_a, link_shape_rep_a);
                SidePanel::left("proximity_visualizer").show(egui_contexts.ctx_mut(), |ui| {
                    ui.heading("Pairwise Distances");
                    c.action_chain_proximity_visualizer(
                        ui,
                        &chain_state_a.state,
                        &chain_state_a.state,
                        &link_shape_mode_a,
                        &link_shape_mode_a,
                        &link_shape_rep_a,
                        &link_shape_rep_a,
                        Some(skips),
                        Some(&stats.averages),
                        &mut selected_idxs,
                        &mut color_change_engine,
                        &mut gizmos,
                    );

                    ui.separator();

                    ComboBox::from_label("Link shape mode")
                        .selected_text(format!("{:?}", link_shape_mode_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Full,
                                "Full",
                            );
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Decomposition,
                                "Decomposition",
                            );
                        });
                    ComboBox::from_label("Link shape rep")
                        .selected_text(format!("{:?}", link_shape_rep_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::ConvexHull,
                                "Convex Hull",
                            );
                            ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::OBB, "OBB");
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::BoundingSphere,
                                "Bounding Sphere",
                            );
                        });

                    ui.separator();

                    if ui.button("Deselect").clicked() {
                        selected_idxs = None;
                    }

                    set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
                });
            })
            .after(S1),
        );

        app
    }

    fn bevy_display_with_other_chain_app(&self, other: &ChainNalgebra) -> App {
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S2;
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S3;

        let mut app = App::new()
            .apollo_bevy_robotics_base(true, false)
            .apollo_bevy_spawn_chain(
                self,
                0,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            )
            .apollo_bevy_spawn_chain(
                other,
                1,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            );

        let arc_robot_a = Arc::new(self.clone());
        let c = BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: arc_robot_a.clone(),
            color_changes: true,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

        let arc_robot_b = Arc::new(other.clone());
        let c = BevyChainSlidersEgui {
            chain_instance_idx: 1,
            chain: arc_robot_b.clone(),
            color_changes: true,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S2).after(S1));

        let c = BevyChainLinkVisibilitySelector {
            chain_instance_idx: 0,
            chain: arc_robot_a.clone(),
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S3).after(S2));

        let c = BevyChainLinkVisibilitySelector {
            chain_instance_idx: 1,
            chain: arc_robot_b.clone(),
        };
        app.add_systems(Update, c.get_system_side_panel_left().after(S3));

        app
    }

    fn bevy_proximity_vis_with_other_chain_app(&self, other: &ChainNalgebra) -> App {
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S1;
        #[derive(SystemSet, PartialEq, Eq, Hash, Clone, Debug)]
        struct S2;

        let mut app = App::new()
            .apollo_bevy_robotics_base(true, false)
            .apollo_bevy_spawn_chain(
                self,
                0,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            )
            .apollo_bevy_spawn_chain(
                other,
                1,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            );

        let arc_chain = Arc::new(self.clone());
        let arc_chain_other = Arc::new(other.clone());

        let c = BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: arc_chain.clone(),
            color_changes: false,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

        let c = BevyChainSlidersEgui {
            chain_instance_idx: 1,
            chain: arc_chain_other.clone(),
            color_changes: false,
        };
        app.add_systems(Update, c.get_system_side_panel_left().in_set(S2).after(S1));

        let mut link_shape_mode_a = LinkShapeMode::Full;
        let mut link_shape_mode_b = LinkShapeMode::Full;
        let mut link_shape_rep_a = LinkShapeRep::ConvexHull;
        let mut link_shape_rep_b = LinkShapeRep::ConvexHull;
        let mut selected_idxs = None;

        let c = BevyChainProximityVisualizer {
            chain_instance_idx_a: 0,
            chain_a: arc_chain.clone(),
            chain_instance_idx_b: 1,
            chain_b: arc_chain_other.clone(),
        }
        .to_raw();

        app.add_systems(
            Update,
            (move |mut egui_contexts: EguiContexts,
                   mut color_change_engine: ResMut<ColorChangeEngine>,
                   mut visibility_change_engine: ResMut<VisibilityChangeEngine>,
                   query: Query<&ChainState>,
                   mut cursor_is_over_egui: ResMut<CursorIsOverEgui>,
                   query2: Query<&Window, With<PrimaryWindow>>,
                   mut gizmos: Gizmos| {
                let chain_state_a = query
                    .iter()
                    .find(|x| x.chain_instance_idx == 0)
                    .expect("error");
                let chain_state_b = query
                    .iter()
                    .find(|x| x.chain_instance_idx == 1)
                    .expect("error");

                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::Off,
                    Signature::new_chain_link_mesh(vec![]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::from_link_shape_mode_and_link_shape_rep(
                                &link_shape_mode_a,
                                &link_shape_rep_a,
                            ),
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::Plain,
                        ),
                        ChainMeshComponent::ChainInstanceIdx(0),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::from_link_shape_mode_and_link_shape_rep(
                                &link_shape_mode_b,
                                &link_shape_rep_b,
                            ),
                        ),
                        ChainMeshComponent::ChainInstanceIdx(1),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));
                visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(
                    VisibilityChangeRequestType::On,
                    Signature::new_chain_link_mesh(vec![
                        ChainMeshComponent::ChainMeshesRepresentation(
                            ChainMeshesRepresentation::Plain,
                        ),
                        ChainMeshComponent::ChainInstanceIdx(1),
                        ChainMeshComponent::MeshType(MeshType::OBJ),
                    ]),
                ));

                SidePanel::left("proximity_visualizer").show(egui_contexts.ctx_mut(), |ui| {
                    ui.heading("Pairwise Distances");
                    c.action_chain_proximity_visualizer(
                        ui,
                        &chain_state_a.state,
                        &chain_state_b.state,
                        &link_shape_mode_a,
                        &link_shape_mode_b,
                        &link_shape_rep_a,
                        &link_shape_rep_b,
                        None,
                        None,
                        &mut selected_idxs,
                        &mut color_change_engine,
                        &mut gizmos,
                    );

                    ui.separator();

                    ComboBox::from_label("Link shape mode A")
                        .selected_text(format!("{:?}", link_shape_mode_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Full,
                                "Full",
                            );
                            ui.selectable_value(
                                &mut link_shape_mode_a,
                                LinkShapeMode::Decomposition,
                                "Decomposition",
                            );
                        });
                    ComboBox::from_label("Link shape rep A")
                        .selected_text(format!("{:?}", link_shape_rep_a))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::ConvexHull,
                                "Convex Hull",
                            );
                            ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::OBB, "OBB");
                            ui.selectable_value(
                                &mut link_shape_rep_a,
                                LinkShapeRep::BoundingSphere,
                                "Bounding Sphere",
                            );
                        });
                    ComboBox::from_label("Link shape mode B")
                        .selected_text(format!("{:?}", link_shape_mode_b))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_mode_b,
                                LinkShapeMode::Full,
                                "Full",
                            );
                            ui.selectable_value(
                                &mut link_shape_mode_b,
                                LinkShapeMode::Decomposition,
                                "Decomposition",
                            );
                        });
                    ComboBox::from_label("Link shape rep B")
                        .selected_text(format!("{:?}", link_shape_rep_b))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut link_shape_rep_b,
                                LinkShapeRep::ConvexHull,
                                "Convex Hull",
                            );
                            ui.selectable_value(&mut link_shape_rep_b, LinkShapeRep::OBB, "OBB");
                            ui.selectable_value(
                                &mut link_shape_rep_b,
                                LinkShapeRep::BoundingSphere,
                                "Bounding Sphere",
                            );
                        });

                    ui.separator();

                    if ui.button("Deselect").clicked() {
                        selected_idxs = None;
                    }

                    set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
                });
            })
            .after(S2),
        );

        app
    }

    fn bevy_motion_playback_app<I: InterpolatorTraitLite + 'static>(&self, interpolator: I) -> App {
        let mut app = App::new()
            .apollo_bevy_robotics_base(true, false)
            .apollo_bevy_spawn_chain(
                self,
                0,
                ISE3q::identity(),
                get_default_mesh_specs(),
                &PathBuf::new_from_default_apollo_bevy_assets_dir(),
            );

        let c = BevyChainMotionPlayback {
            chain_instance_idx: 0,
        };
        app.add_systems(Update, c.get_system_bottom_panel(Arc::new(interpolator)));

        app
    }
}

pub fn get_default_mesh_specs() -> Vec<(ChainMeshesRepresentation, MeshType, BaseVisibility)> {
    return vec![
        (
            ChainMeshesRepresentation::Plain,
            MeshType::OBJ,
            BaseVisibility::On,
        ),
        (
            ChainMeshesRepresentation::Plain,
            MeshType::GLB,
            BaseVisibility::Off,
        ),
        (
            ChainMeshesRepresentation::ConvexHull,
            MeshType::OBJ,
            BaseVisibility::Off,
        ),
        (
            ChainMeshesRepresentation::ConvexHull,
            MeshType::GLB,
            BaseVisibility::Off,
        ),
        (
            ChainMeshesRepresentation::ConvexDecomposition,
            MeshType::OBJ,
            BaseVisibility::Off,
        ),
        (
            ChainMeshesRepresentation::ConvexDecomposition,
            MeshType::GLB,
            BaseVisibility::Off,
        ),
    ];
}

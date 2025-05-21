use std::env;
use std::path::PathBuf;
use bevy::app::{App, AppExit, Update};
use bevy::prelude::{EventWriter, Gizmos, IntoSystemConfigs, Query, ResMut, SystemSet, Window, With};
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{Color32, ComboBox, RichText, SidePanel};
use bevy_egui::EguiContexts;
use apollo_rust_bevy::{ApolloBevyTrait, get_default_mesh_specs};
use apollo_rust_bevy::apollo_bevy_utils::chain::{BevyChainProximityVisualizerRaw, BevyChainSlidersEguiRaw, ChainMeshesRepresentation, ChainState};
use apollo_rust_bevy::apollo_bevy_utils::colors::ColorChangeEngine;
use apollo_rust_bevy::apollo_bevy_utils::egui::{CursorIsOverEgui, set_cursor_is_over_egui_default};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_bevy::apollo_bevy_utils::signatures::{ChainMeshComponent, Signature};
use apollo_rust_bevy::apollo_bevy_utils::visibility::{VisibilityChangeEngine, VisibilityChangeRequest, VisibilityChangeRequestType};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_file::traits::ToJsonString;
use apollo_rust_linalg::{dmatrix_from_2dvec, dmatrix_to_2dvec};
use apollo_rust_preprocessor::PreprocessorModule;
use apollo_rust_proximity_parry::double_group_queries::{ConvertToAverageDistancesTrait, DoubleGroupProximityQueryMode};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_preprocessor::robot_modules_preprocessor::CombinedRobot;
use apollo_rust_robotics_core::modules_runtime::link_shapes_distance_statistics_nalgebra_module::ApolloLinkShapesDistanceStatisticsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::modules_runtime::link_shapes_simple_skips_nalgebra_module::ApolloLinkShapesSimpleSkipsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_robotics_core::robot_functions::robot_proximity_functions::RobotProximityFunctions;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 2);
    let chain_name = &args[1];
    let ss = if let Some(s) = ResourcesRootDirectory::new_from_default_apollo_robots_dir().get_subdirectory_option(chain_name) {
        s
    } else if let Some(s) = ResourcesRootDirectory::new_from_default_apollo_environments_dir().get_subdirectory_option(chain_name) {
        s
    } else {
        panic!("not found")
    };
    let s = &ss;

    let urdf_module = ApolloURDFModule::load_or_build(s, false).expect("error");
    let urdf_nalgebra_module = ApolloURDFNalgebraModule::from_urdf_module(&urdf_module);
    let dof_module = ApolloDOFModule::load_or_build(s, false).expect("error");
    let chain_module = ApolloChainModule::load_or_build(s, false).expect("error");
    let bounds_module = ApolloBoundsModule::load_or_build(s, false).expect("error");
    let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(s, false).expect("error");
    let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
    let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
    let link_shapes_approximations_module = ApolloLinkShapesApproximationsModule::load_or_build(s, false).expect("error");
    let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);
    let link_shapes_simple_skips_module = ApolloLinkShapesSimpleSkipsModule::load_or_build(s, false).expect("error");
    let link_shapes_simple_skips_module = ApolloLinkShapesSimpleSkipsNalgebraModule::from_link_shapes_simple_skips_module(&link_shapes_simple_skips_module);
    let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsModule::load_or_build(s, false).expect("error");
    let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsNalgebraModule::from_link_shapes_distance_statistics_module(&link_shapes_distance_statistics_module);
    let link_shapes_skips_module_result = ApolloLinkShapesSkipsModule::load_from_json(s);

    let mut app = App::new()
        .apollo_bevy_robotics_base(true)
        .apollo_bevy_spawn_chain_raw(s, &urdf_nalgebra_module, &chain_module, &dof_module, &plain_meshes_module, &convex_hull_meshes_module, &convex_decomposition_meshes_module, &link_shapes_approximations_module, 0, ISE3q::identity(), get_default_mesh_specs(), &PathBuf::new_from_default_apollo_bevy_assets_dir());

    #[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
    struct S1;
    #[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
    struct S2;

    let c = BevyChainSlidersEguiRaw {
        chain_instance_idx: 0,
        urdf_module: urdf_nalgebra_module.clone(),
        chain_module: chain_module.clone(),
        dof_module: dof_module.clone(),
        bounds_module: bounds_module.clone(),
        color_changes: false,
    };
    app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

    let c = BevyChainProximityVisualizerRaw {
        chain_instance_idx_a: 0,
        chain_instance_idx_b: 0,
        urdf_module_a: urdf_nalgebra_module.clone(),
        urdf_module_b: urdf_nalgebra_module.clone(),
        chain_module_a: chain_module.clone(),
        chain_module_b: chain_module.clone(),
        dof_module_a: dof_module.clone(),
        dof_module_b: dof_module.clone(),
        link_shapes_module_a: link_shapes_module.clone(),
        link_shapes_module_b: link_shapes_module.clone(),
        double_group_proximity_query_mode: DoubleGroupProximityQueryMode::SkipSymmetricalPairs,
    };

    let mut link_shape_mode_a = LinkShapeMode::Full;
    let mut link_shape_rep_a = LinkShapeRep::ConvexHull;
    let mut selected_idxs = None;

    let mut full_convex_hulls_skips;
    let mut full_obbs_skips;
    let mut full_bounding_spheres_skips;
    let mut decomposition_convex_hulls_skips;
    let mut decomposition_obbs_skips;
    let mut decomposition_bounding_spheres_skips;

    match &link_shapes_skips_module_result {
        Ok(link_shapes_skips_module) => {
            full_convex_hulls_skips = dmatrix_from_2dvec(&link_shapes_skips_module.full_convex_hulls_skips.clone());
            full_obbs_skips = dmatrix_from_2dvec(&link_shapes_skips_module.full_obbs_skips.clone());
            full_bounding_spheres_skips = dmatrix_from_2dvec(&link_shapes_skips_module.full_bounding_spheres_skips.clone());
            decomposition_convex_hulls_skips = dmatrix_from_2dvec(&link_shapes_skips_module.decomposition_convex_hulls_skips.clone());
            decomposition_obbs_skips = dmatrix_from_2dvec(&link_shapes_skips_module.decomposition_obbs_skips.clone());
            decomposition_bounding_spheres_skips = dmatrix_from_2dvec(&link_shapes_skips_module.decomposition_bounding_spheres_skips.clone());
        }
        Err(_) => {
            full_convex_hulls_skips = link_shapes_simple_skips_module.full_convex_hulls_simple_skips.clone();
            full_obbs_skips = link_shapes_simple_skips_module.full_obbs_simple_skips.clone();
            full_bounding_spheres_skips = link_shapes_simple_skips_module.full_bounding_spheres_simple_skips.clone();
            decomposition_convex_hulls_skips = link_shapes_simple_skips_module.decomposition_convex_hulls_simple_skips.clone();
            decomposition_obbs_skips = link_shapes_simple_skips_module.decomposition_obbs_simple_skips.clone();
            decomposition_bounding_spheres_skips = link_shapes_simple_skips_module.decomposition_bounding_spheres_simple_skips.clone();
        }
    }

    let mut original_full_convex_hulls_skips = link_shapes_simple_skips_module.full_convex_hulls_simple_skips.clone();
    let mut original_full_obbs_skips = link_shapes_simple_skips_module.full_obbs_simple_skips.clone();
    let mut original_full_bounding_spheres_skips = link_shapes_simple_skips_module.full_bounding_spheres_simple_skips.clone();
    let mut original_decomposition_convex_hulls_skips = link_shapes_simple_skips_module.decomposition_convex_hulls_simple_skips.clone();
    let mut original_decomposition_obbs_skips = link_shapes_simple_skips_module.decomposition_obbs_simple_skips.clone();
    let mut original_decomposition_bounding_spheres_skips = link_shapes_simple_skips_module.decomposition_bounding_spheres_simple_skips.clone();

    let combined_robot = CombinedRobot::load(s);
    match &combined_robot {
        Ok(combined_robot) => {
            let info = combined_robot.to_combined_sub_chain_info(s);

            for (chain_idx, attached_robot) in combined_robot.attached_robots().iter().enumerate() {
                let ss = ResourcesRootDirectory::new(s.root_directory.clone(), s.resources_type.clone()).get_subdirectory(attached_robot.robot_name());

                let ss_convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(&ss, false).expect("error");
                let ss_convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(&ss, false).expect("error");
                let ss_link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(&ss, &ss_convex_hull_meshes_module, &ss_convex_decomposition_meshes_module);
                let ss_link_shapes_skips_module = ApolloLinkShapesSkipsModule::load_or_build(&ss, false).expect("error");

                let num_full = ss_link_shapes_skips_module.full_convex_hulls_skips.len();
                let num_decomposition = ss_link_shapes_skips_module.decomposition_convex_hulls_skips.len();

                for i in 0..num_full {
                    let sub_link_idx_i = ss_link_shapes_module.full_shape_idx_to_link_idx[i];
                    let combined_link_idx_i = info.sub_chain_and_link_idx_to_combined_chain_link_idx[chain_idx][sub_link_idx_i].clone();
                    let combined_shape_idx_i = link_shapes_module.link_idx_to_full_shape_idx[combined_link_idx_i].unwrap();
                    for j in 0..num_full {
                        let sub_link_idx_j = ss_link_shapes_module.full_shape_idx_to_link_idx[j];
                        let combined_link_idx_j = info.sub_chain_and_link_idx_to_combined_chain_link_idx[chain_idx][sub_link_idx_j].clone();
                        let combined_shape_idx_j = link_shapes_module.link_idx_to_full_shape_idx[combined_link_idx_j].unwrap();

                        let sub_skip = ss_link_shapes_skips_module.full_bounding_spheres_skips[i][j];
                        if sub_skip {
                            full_bounding_spheres_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            full_bounding_spheres_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_full_bounding_spheres_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_full_bounding_spheres_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                        let sub_skip = ss_link_shapes_skips_module.full_obbs_skips[i][j];
                        if sub_skip {
                            full_obbs_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            full_obbs_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_full_obbs_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_full_obbs_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                        let sub_skip = ss_link_shapes_skips_module.full_convex_hulls_skips[i][j];
                        if sub_skip {
                            full_convex_hulls_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            full_convex_hulls_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_full_convex_hulls_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_full_convex_hulls_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                    }
                }

                for i in 0..num_decomposition {
                    let (sub_link_idx_i, aa) = ss_link_shapes_module.decomposition_shape_idx_to_link_idx_and_link_sub_idx[i];
                    let combined_link_idx_i = info.sub_chain_and_link_idx_to_combined_chain_link_idx[chain_idx][sub_link_idx_i].clone();
                    let combined_shape_idxs_i = link_shapes_module.link_idx_to_decomposition_shape_idxs[combined_link_idx_i].clone();
                    let combined_shape_idx_i = combined_shape_idxs_i[aa];
                    for j in 0..num_decomposition {
                        let (sub_link_idx_j, bb) = ss_link_shapes_module.decomposition_shape_idx_to_link_idx_and_link_sub_idx[j];
                        let combined_link_idx_j = info.sub_chain_and_link_idx_to_combined_chain_link_idx[chain_idx][sub_link_idx_j].clone();
                        let combined_shape_idxs_j = link_shapes_module.link_idx_to_decomposition_shape_idxs[combined_link_idx_j].clone();
                        let combined_shape_idx_j = combined_shape_idxs_j[bb];

                        let sub_skip = ss_link_shapes_skips_module.decomposition_bounding_spheres_skips[i][j];
                        if sub_skip {
                            decomposition_bounding_spheres_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            decomposition_bounding_spheres_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_decomposition_bounding_spheres_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_decomposition_bounding_spheres_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                        let sub_skip = ss_link_shapes_skips_module.decomposition_obbs_skips[i][j];
                        if sub_skip {
                            decomposition_obbs_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            decomposition_obbs_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_decomposition_obbs_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_decomposition_obbs_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                        let sub_skip = ss_link_shapes_skips_module.decomposition_convex_hulls_skips[i][j];
                        if sub_skip {
                            decomposition_convex_hulls_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            decomposition_convex_hulls_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                            original_decomposition_convex_hulls_skips[(combined_shape_idx_i, combined_shape_idx_j)] = true;
                            original_decomposition_convex_hulls_skips[(combined_shape_idx_j, combined_shape_idx_i)] = true;
                        }

                    }
                }
            }
        }
        Err(_) => { }
    }

    let link_shapes_distance_statistics_module_clone = link_shapes_distance_statistics_module.clone();
    let link_shapes_module_clone = link_shapes_module.clone();

    app.add_systems(Update, (move |mut exit: EventWriter<AppExit>, mut egui_contexts: EguiContexts, mut color_change_engine: ResMut<ColorChangeEngine>, mut visibility_change_engine: ResMut<VisibilityChangeEngine>, query: Query<&ChainState>, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window, With<PrimaryWindow>>, mut gizmos: Gizmos| {
        let chain_state_a = query.iter().find(|x| x.chain_instance_idx == c.chain_instance_idx_a).expect("error");
        let chain_state_b = query.iter().find(|x| x.chain_instance_idx == c.chain_instance_idx_b).expect("error");

        let averages = &link_shapes_distance_statistics_module_clone.get_stats(&link_shape_rep_a, &link_shape_mode_a).averages;

        let skips = match &link_shape_mode_a {
            LinkShapeMode::Full => {
                match &link_shape_rep_a {
                    LinkShapeRep::ConvexHull => { &mut full_convex_hulls_skips }
                    LinkShapeRep::OBB => { &mut full_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &mut full_bounding_spheres_skips }
                }
            }
            LinkShapeMode::Decomposition => {
                match &link_shape_rep_a {
                    LinkShapeRep::ConvexHull => { &mut decomposition_convex_hulls_skips }
                    LinkShapeRep::OBB => { &mut decomposition_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &mut decomposition_bounding_spheres_skips }
                }
            }
        };
        let original_skips = match &link_shape_mode_a {
            LinkShapeMode::Full => {
                match &link_shape_rep_a {
                    LinkShapeRep::ConvexHull => { &original_full_convex_hulls_skips }
                    LinkShapeRep::OBB => { &original_full_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &original_full_bounding_spheres_skips }
                }
            }
            LinkShapeMode::Decomposition => {
                match &link_shape_rep_a {
                    LinkShapeRep::ConvexHull => { &original_decomposition_convex_hulls_skips }
                    LinkShapeRep::OBB => { &original_decomposition_obbs_skips }
                    LinkShapeRep::BoundingSphere => { &original_decomposition_bounding_spheres_skips }
                }
            }
        };

        visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::Off, Signature::new_chain_link_mesh(vec![ ])));
        visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::On, Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::from_link_shape_mode_and_link_shape_rep(&link_shape_mode_a, &link_shape_rep_a)), ChainMeshComponent::ChainInstanceIdx(c.chain_instance_idx_a), ChainMeshComponent::MeshType(MeshType::OBJ)])));
        visibility_change_engine.add_momentary_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::On, Signature::new_chain_link_mesh(vec![ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::Plain), ChainMeshComponent::ChainInstanceIdx(c.chain_instance_idx_a), ChainMeshComponent::MeshType(MeshType::OBJ)])));

        SidePanel::left("proximity_visualizer").show(egui_contexts.ctx_mut(), |ui| {
            ui.heading("Pairwise Distances Raw");
            c.action_chain_proximity_visualizer(ui, &chain_state_a.state, &chain_state_b.state, &link_shape_mode_a, &link_shape_mode_a, &link_shape_rep_a, &link_shape_rep_a, Some(skips), Some(averages), &mut selected_idxs, &mut color_change_engine, &mut gizmos);

            ui.separator();

            ComboBox::from_label("Link shape mode").selected_text(format!("{:?}", link_shape_mode_a)).show_ui(ui, |ui| {
                ui.selectable_value(&mut link_shape_mode_a, LinkShapeMode::Full, "Full");
                ui.selectable_value(&mut link_shape_mode_a, LinkShapeMode::Decomposition, "Decomposition");
            });
            ComboBox::from_label("Link shape rep").selected_text(format!("{:?}", link_shape_rep_a)).show_ui(ui, |ui| {
                ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::ConvexHull, "Convex Hull");
                ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::OBB, "OBB");
                ui.selectable_value(&mut link_shape_rep_a, LinkShapeRep::BoundingSphere, "Bounding Sphere");
            });

            ui.separator();

            if ui.button("Deselect").clicked() { selected_idxs = None; }

            set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
        });

        let mut ee = false;
        SidePanel::left("another panel").show(egui_contexts.ctx_mut(), |ui| {
            ui.add_space(30.0);

            if ui.button(RichText::new("Set selected as skip (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() {
                match selected_idxs {
                    None => {}
                    Some((x, y)) => {
                        let i = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(x.0, x.1, &link_shape_mode_a).expect("error");
                        let j = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(y.0, y.1, &link_shape_mode_a).expect("error");
                        skips[(i,j)] = true;
                    }
                }
            }

            ui.separator();
            if ui.button(RichText::new("Use selected as raw distance cutoff (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() && selected_idxs.is_some() {
                let selected_idxs = selected_idxs.unwrap();
                let x = selected_idxs.0;
                let y = selected_idxs.1;
                let i = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(x.0, x.1, &link_shape_mode_a).expect("error");
                let j = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(y.0, y.1, &link_shape_mode_a).expect("error");
                let link_poses = RobotKinematicsFunctions::fk(&chain_state_a.state, &urdf_nalgebra_module, &chain_module, &dof_module);
                let res = RobotProximityFunctions::self_contact(&link_shapes_module, &link_poses, link_shape_mode_a, link_shape_rep_a, None, false, f64::INFINITY);
                let idx = res.shape_idxs.iter().position(|x| x.0 == i && x.1 == j).unwrap();
                let cutoff_distance = res.outputs[idx].unwrap().dist;
                res.outputs.iter().zip(res.shape_idxs.iter()).for_each(|(x, y)| {
                    let x = x.unwrap();
                    if x.dist <= cutoff_distance { skips[(y.0, y.1)] = true; }
                });
            }
            ui.label("If the button above is pressed, all pairs that have a raw distance that is less than or equal to the selected pair will be set as a skip");
            ui.separator();

            if ui.button(RichText::new("Use selected as wrt average distance cutoff (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() && selected_idxs.is_some() {
                let selected_idxs = selected_idxs.unwrap();
                let x = selected_idxs.0;
                let y = selected_idxs.1;
                let i = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(x.0, x.1, &link_shape_mode_a).expect("error");
                let j = link_shapes_module_clone.get_shape_idx_from_link_idx_and_subcomponent_idx(y.0, y.1, &link_shape_mode_a).expect("error");
                let link_poses = RobotKinematicsFunctions::fk(&chain_state_a.state, &urdf_nalgebra_module, &chain_module, &dof_module);
                let res = RobotProximityFunctions::self_contact(&link_shapes_module, &link_poses, link_shape_mode_a, link_shape_rep_a, None, false, f64::INFINITY);
                let res = res.to_average_distances(&averages);
                let idx = res.shape_idxs.iter().position(|x| x.0 == i && x.1 == j).unwrap();
                let cutoff_distance = res.outputs[idx].unwrap().dist;
                res.outputs.iter().zip(res.shape_idxs.iter()).for_each(|(x, y)| {
                    let x = x.unwrap();
                    if x.dist <= cutoff_distance { skips[(y.0, y.1)] = true; }
                });
            }
            ui.label("If the button above is pressed, all pairs that have a distance with respect to average that is less than or equal to the selected pair will be set as a skip");
            ui.separator();

            if ui.button(RichText::new("Reset skips (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() {
                *skips = original_skips.clone();
            }

            ui.add_space(40.0);

            if ui.button(RichText::new("Save and exit (double click)").size(16.0)).double_clicked() {
                ee = true;
            }
            // ui.separator();
            // ui.label("Raw distance cutoff");
            // ui.add(Slider::new(&mut raw_distance_cutoff, 0.001..=1.0));
            // if ui.button(RichText::new("Set skips (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() {
            // }
            // ui.label(RichText::new("(all shape pairs with a raw distance less than cutoff will be set as skip)"));
            // ui.separator();

            // ui.separator();
            // ui.label("With respect to average distance cutoff");
            // ui.add(Slider::new(&mut average_distance_cutoff, 0.001..=1.0));
            // if ui.button(RichText::new("Set skips (double click)").color(Color32::from_rgb(255, 100, 0))).double_clicked() {
            //
            // }
            // ui.label(RichText::new("(all shape pairs with a distance with respect to average less than cutoff will be set as skip)"));
            // ui.separator();

            set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
        });

        if ee {
            println!("{}", dmatrix_to_2dvec(&full_convex_hulls_skips).to_json_string());
            println!("{}", dmatrix_to_2dvec(&full_obbs_skips).to_json_string());
            println!("{}", dmatrix_to_2dvec(&full_bounding_spheres_skips).to_json_string());
            println!("{}", dmatrix_to_2dvec(&decomposition_convex_hulls_skips).to_json_string());
            println!("{}", dmatrix_to_2dvec(&decomposition_obbs_skips).to_json_string());
            println!("{}", dmatrix_to_2dvec(&decomposition_bounding_spheres_skips).to_json_string());

            exit.send(AppExit::Success);
        }

    }).in_set(S2).after(S1));

    app.run();
}
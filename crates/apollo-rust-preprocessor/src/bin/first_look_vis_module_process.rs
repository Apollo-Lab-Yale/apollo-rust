use std::env;
use std::path::PathBuf;
use bevy::app::App;
use bevy::prelude::{AppExit, EventWriter, IntoSystemConfigs, Query, ResMut, SystemSet, Update, Window, With};
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{Color32, RichText, TopBottomPanel};
use bevy_egui::EguiContexts;
use apollo_rust_bevy::apollo_bevy_utils::chain::{BevyChainLinkVisibilitySelectorRaw, BevyChainSlidersEguiRaw};
use apollo_rust_bevy::apollo_bevy_utils::egui::{CursorIsOverEgui, set_cursor_is_over_egui_default};
use apollo_rust_bevy::{ApolloBevyTrait, get_default_mesh_specs};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_file::traits::ToJsonString;
use apollo_rust_preprocessor::PreprocessorModule;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let args: Vec<String> = env::args().collect();
    assert_eq!(args.len(), 2);
    let chain_name = &args[1];
    let ss = if let Some(s) = ResourcesRootDirectory::new_from_default_apollo_robots_directory().get_subdirectory_option(chain_name) {
        s
    } else if let Some(s) = ResourcesRootDirectory::new_from_default_apollo_environments_directory().get_subdirectory_option(chain_name) {
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

    let mut app = App::new()
        .apollo_bevy_robotics_base(true)
        .apollo_bevy_spawn_robot_raw(s, &urdf_nalgebra_module, &chain_module, &dof_module, &plain_meshes_module, &convex_hull_meshes_module, &convex_decomposition_meshes_module, &link_shapes_approximations_module, 0, ISE3q::identity(), get_default_mesh_specs(), &PathBuf::new_from_default_apollo_bevy_assets_dir());

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
        color_changes: true,
    };
    app.add_systems(Update, c.get_system_side_panel_left().in_set(S1));

    let c = BevyChainLinkVisibilitySelectorRaw::new(0, urdf_nalgebra_module.clone(), chain_module.clone());
    app.add_systems(Update, c.get_system_side_panel_left().in_set(S2).after(S1));

    app.add_systems(Update, (move |mut exit: EventWriter<AppExit>, mut egui_contexts: EguiContexts, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window, With<PrimaryWindow>>| {
        TopBottomPanel::bottom("bottom_panel").default_height(70.0).show(egui_contexts.ctx_mut(), |ui| {
            ui.horizontal(|ui| {
                if ui.button(RichText::new("Verified").color(Color32::GREEN).size(25.0)).clicked() {
                    let json_string = true.to_json_string();
                    println!("{}", json_string);
                    exit.send(AppExit::Success);
                }
                ui.separator();
                if ui.button(RichText::new("Not Verified").color(Color32::RED).size(25.0)).clicked() {
                    let json_string = false.to_json_string();
                    println!("{}", json_string);
                    exit.send(AppExit::Success);
                }
            });
            set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
        });
    }).after(S2));

    /*
    let c = BevyChainSlidersEguiRaw {
        chain_instance_idx: 0,
        urdf_module: urdf_nalgebra_module.clone(),
        chain_module: chain_module.clone(),
        dof_module: dof_module.clone(),
        bounds_module: bounds_module.clone(),
        color_changes: true,
    };
    app.add_systems(Update, move |mut exit: EventWriter<AppExit>, mut color_change_engine: ResMut<ColorChangeEngine>, mut egui_contexts: EguiContexts, mut query: Query<&mut ChainState>, mut cursor_is_over_egui: ResMut<CursorIsOverEgui>, query2: Query<&Window, With<PrimaryWindow>>| {
        SidePanel::left(format!("chain_sliders_side_panel_chain_instance_idx_{}", c.chain_instance_idx))
            .show(egui_contexts.ctx_mut(),  |ui| {
                c.action_chain_sliders_egui(&mut color_change_engine, &mut query, ui);
                if ui.button("Verified").clicked() {
                    let json_string = true.to_json_string();
                    println!("{}", json_string);
                    exit.send(AppExit::Success);
                }
                ui.separator();
                if ui.button("Not Verified").clicked() {
                    let json_string = false.to_json_string();
                    println!("{}", json_string);
                    exit.send(AppExit::Success);
                }
                set_cursor_is_over_egui_default(ui, &mut cursor_is_over_egui, &query2);
            });
    });
    */

    app.run();
}
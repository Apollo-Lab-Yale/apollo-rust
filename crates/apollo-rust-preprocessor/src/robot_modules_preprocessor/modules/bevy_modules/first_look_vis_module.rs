use std::path::PathBuf;
use bevy::prelude::{App, Update};
use apollo_rust_bevy::{ApolloBevyTrait, get_default_mesh_specs};
use apollo_rust_bevy::apollo_bevy_utils::chain::BevyChainSlidersEguiRaw;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::bevy_modules::first_look_vis_module::ApolloFirstLookVisModule;
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;


impl PreprocessorModule for ApolloFirstLookVisModule {
    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "bevy_modules/first_look_vis_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false).expect("error");
        let urdf_nalgebra_module = ApolloURDFNalgebraModule::from_urdf_module(&urdf_module);
        let dof_module = ApolloDOFModule::load_or_build(s, false).expect("error");
        let chain_module = ApolloChainModule::load_or_build(s, false).expect("error");
        let bounds_module = ApolloBoundsModule::load_or_build(s, false).expect("error");
        let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(s, false).expect("error");
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");

        let mut app = App::new()
            .apollo_bevy_base()
            .apollo_bevy_pan_orbit_three_style_camera()
            .apollo_bevy_starter_lights()
            .apollo_bevy_robotics_scene_visuals_start()
            .apollo_bevy_spawn_robot_raw(s, &urdf_nalgebra_module, &chain_module, &dof_module, &plain_meshes_module, &convex_hull_meshes_module, &convex_decomposition_meshes_module, 0, ISE3q::identity(), get_default_mesh_specs(), &PathBuf::new_from_default_apollo_bevy_assets_dir());

        let c = BevyChainSlidersEguiRaw {
            chain_instance_idx: 0,
            urdf_module: urdf_nalgebra_module.clone(),
            chain_module: chain_module.clone(),
            dof_module: dof_module.clone(),
            bounds_module: bounds_module.clone(),
        };
        // app.add_systems(Update, c.get_system_side_panel_left(Some(|ui: &Ui, s: &BevyChainSlidersEguiRaw| { })));

        todo!()
    }
}
use apollo_rust_robot_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::ApolloLinkShapesModule;
use apollo_rust_robotics_core::RobotPreprocessorSingleRobotDirectory;
use crate::RobotPreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

impl RobotPreprocessorModule for ApolloLinkShapesMaxDistanceFromOriginModule {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String {
        "link_shape_max_distance_from_origin_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);

        let mut full_convex_hulls_maximum_distances = vec![];
        let mut full_obbs_maximum_distances = vec![];
        let mut full_bounding_spheres_maximum_distances = vec![];
        let mut decomposition_convex_hulls_maximum_distances = vec![];
        let mut decomposition_obbs_maximum_distances = vec![];
        let mut decomposition_bounding_spheres_maximum_distances = vec![];

        link_shapes_module.link_idx_to_full_shape_idx().iter().for_each(|x| {
            match x {
                None => {
                    full_convex_hulls_maximum_distances.push(None);
                    full_obbs_maximum_distances.push(None);
                    full_bounding_spheres_maximum_distances.push(None);
                }
                Some(full_shape_idx) => {
                    full_convex_hulls_maximum_distances.push(Some(link_shapes_module.full_convex_hulls[*full_shape_idx].calculate_max_dis_from_origin_to_point_on_shape()));
                    full_obbs_maximum_distances.push(Some(link_shapes_module.full_obbs[*full_shape_idx].calculate_max_dis_from_origin_to_point_on_shape()));
                    full_bounding_spheres_maximum_distances.push(Some(link_shapes_module.full_bounding_spheres[*full_shape_idx].calculate_max_dis_from_origin_to_point_on_shape()));
                }
            }
        });

        progress_bar.update_with_percentage_preset(50.0);

        link_shapes_module.link_idx_to_decomposition_shape_idxs().iter().for_each(|x| {
            let mut tmp1 = vec![];
            let mut tmp2 = vec![];
            let mut tmp3 = vec![];
            x.iter().for_each(|decomposition_shape_idx| {
                tmp1.push(link_shapes_module.decomposition_convex_hulls[*decomposition_shape_idx].calculate_max_dis_from_origin_to_point_on_shape());
                tmp2.push(link_shapes_module.decomposition_obbs[*decomposition_shape_idx].calculate_max_dis_from_origin_to_point_on_shape());
                tmp3.push(link_shapes_module.decomposition_bounding_spheres[*decomposition_shape_idx].calculate_max_dis_from_origin_to_point_on_shape());
            });

            decomposition_convex_hulls_maximum_distances.push(tmp1);
            decomposition_obbs_maximum_distances.push(tmp2);
            decomposition_bounding_spheres_maximum_distances.push(tmp3);
        });

        progress_bar.done_preset();
        return Ok(ApolloLinkShapesMaxDistanceFromOriginModule {
            full_convex_hulls_maximum_distances,
            full_obbs_maximum_distances,
            full_bounding_spheres_maximum_distances,
            decomposition_convex_hulls_maximum_distances,
            decomposition_obbs_maximum_distances,
            decomposition_bounding_spheres_maximum_distances,
        });
    }
}
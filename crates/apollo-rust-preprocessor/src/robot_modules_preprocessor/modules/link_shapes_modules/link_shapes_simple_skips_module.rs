
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::{ApolloLinkShapesDistanceStatisticsModule, LinkShapesDistanceStatistics};
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode};
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

impl PreprocessorModule for ApolloLinkShapesSimpleSkipsModule {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "link_shapes_modules/link_shapes_simple_skips_module".to_string()
    }

    fn current_version() -> String {
        "0.0.5".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let link_shapes_distance_stats_module = ApolloLinkShapesDistanceStatisticsModule::load_or_build(s, false).expect("error");
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);

        let full_convex_hulls_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.full_convex_hulls, &link_shapes_module, LinkShapeMode::Full);
        let full_obbs_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.full_obbs, &link_shapes_module, LinkShapeMode::Full);
        let full_bounding_spheres_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.full_bounding_spheres, &link_shapes_module, LinkShapeMode::Full);
        let decomposition_convex_hulls_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.decomposition_convex_hulls, &link_shapes_module, LinkShapeMode::Decomposition);
        let decomposition_obbs_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.decomposition_obbs, &link_shapes_module, LinkShapeMode::Decomposition);
        let decomposition_bounding_spheres_simple_skips = get_simple_skips_from_link_shapes_distance_stats(&link_shapes_distance_stats_module.decomposition_bounding_spheres, &link_shapes_module, LinkShapeMode::Decomposition);

        progress_bar.done_preset();
        Ok(Self {
            full_convex_hulls_simple_skips,
            full_obbs_simple_skips,
            full_bounding_spheres_simple_skips,
            decomposition_convex_hulls_simple_skips,
            decomposition_obbs_simple_skips,
            decomposition_bounding_spheres_simple_skips,
        })
    }
}

pub (crate) fn get_simple_skips_from_link_shapes_distance_stats(stats: &LinkShapesDistanceStatistics, link_shapes_module: &ApolloLinkShapesModule, shape_mode: LinkShapeMode) -> Vec<Vec<bool>> {
    let n = stats.maximums.len();

    let mut out = vec![vec![false; n]; n];

    for i in 0..n {
        for j in 0..n {
            let maximum = stats.maximums[i][j];
            // let minimum = stats.minimums[i][j];
            // let average = stats.averages[i][j];

            let mut skip = false;
            if maximum <= 0.001 { skip = true; }
            // if average <= 0.0 { skip = true; }
            // if minimum >= 0.0 { skip = true; }

            let a = link_shapes_module.get_link_idx_and_subcomponent_idx_from_shape_idx(i, &shape_mode);
            let b = link_shapes_module.get_link_idx_and_subcomponent_idx_from_shape_idx(j, &shape_mode);
            if a.0 == b.0 { skip = true; }

            out[i][j] = skip;
        }
    }

    out
}
use apollo_rust_proximity::proxima::proxima2::{get_lladis_taylor_series_error_dataset, LieAlgMode, PolynomialFit, quantile_optimization};
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_lie_alg_error_models_module::ApolloLinkShapesLieAlgErrorModelsModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::ApolloLinkShapesModule;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

impl PreprocessorModule for ApolloLinkShapesLieAlgErrorModelsModule {
    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "link_shapes_modules/link_shapes_lie_alg_error_models_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);
        let num_full_shapes = link_shapes_module.full_convex_hulls.len();
        let num_decomposition_shapes = link_shapes_module.decomposition_convex_hulls.len();

        let mut full_convex_hulls_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut full_obbs_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut full_bounding_spheres_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut decomposition_convex_hulls_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];
        let mut decomposition_obbs_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];
        let mut decomposition_bounding_spheres_standard_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];

        let mut full_convex_hulls_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut full_obbs_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut full_bounding_spheres_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_full_shapes ]; num_full_shapes];
        let mut decomposition_convex_hulls_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];
        let mut decomposition_obbs_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];
        let mut decomposition_bounding_spheres_pseudo_models = vec![vec![ ([0.,0.,0.], [0.,0.,0.]); num_decomposition_shapes ]; num_decomposition_shapes];

        let shape_vecs = vec![
            &link_shapes_module.full_convex_hulls,
            &link_shapes_module.full_obbs,
            &link_shapes_module.full_bounding_spheres,
            &link_shapes_module.decomposition_convex_hulls,
            &link_shapes_module.decomposition_obbs,
            &link_shapes_module.decomposition_bounding_spheres
        ];

        for (idx, shape_vec) in shape_vecs.iter().enumerate() {
            let num_shapes = shape_vec.len();
            
            for i in 0..num_shapes {
                'l: for j in 0..num_shapes {
                    if j >= i { continue 'l; }
                    
                    let shape_a = &shape_vec[i];
                    let shape_b = &shape_vec[j];
                    
                    let ds1 = get_lladis_taylor_series_error_dataset(shape_a, shape_b, LieAlgMode::Standard, 100, 100, 2.0, 3.0);
                    let ds2 = get_lladis_taylor_series_error_dataset(shape_a, shape_b, LieAlgMode::Pseudo, 100, 100, 2.0, 3.0);

                    let lb1 = quantile_optimization(0.01, &ds1, &PolynomialFit::Cubic).0;
                    let ub1 = quantile_optimization(0.99, &ds1, &PolynomialFit::Cubic).0;

                    let lb2 = quantile_optimization(0.01, &ds2, &PolynomialFit::Cubic).0;
                    let ub2 = quantile_optimization(0.99, &ds2, &PolynomialFit::Cubic).0;

                    let model1 = if idx == 0 {
                        &mut full_convex_hulls_standard_models
                    } else if idx == 1 {
                        &mut full_obbs_standard_models
                    } else if idx == 2 {
                        &mut full_bounding_spheres_standard_models
                    } else if idx == 3 {
                        &mut decomposition_convex_hulls_standard_models
                    } else if idx == 4 {
                        &mut decomposition_obbs_standard_models
                    } else if idx == 5 {
                        &mut decomposition_bounding_spheres_standard_models
                    } else { unimplemented!() };
                    model1[(i,j)] = ( [lb1[0], lb1[1], lb1[2]], [ub1[0], ub1[1], ub1[2]] );
                    model1[(j,i)] = ( [lb1[0], lb1[1], lb1[2]], [ub1[0], ub1[1], ub1[2]] );

                    let model2 = if idx == 0 {
                        &mut full_convex_hulls_pseudo_models
                    } else if idx == 1 {
                        &mut full_obbs_pseudo_models
                    } else if idx == 2 {
                        &mut full_bounding_spheres_pseudo_models
                    } else if idx == 3 {
                        &mut decomposition_convex_hulls_pseudo_models
                    } else if idx == 4 {
                        &mut decomposition_obbs_pseudo_models
                    } else if idx == 5 {
                        &mut decomposition_bounding_spheres_pseudo_models
                    } else { unimplemented!() };
                    model2[(i,j)] = ( [lb2[0], lb2[1], lb2[2]], [ub2[0], ub2[1], ub2[2]] );
                    model2[(j,i)] = ( [lb2[0], lb2[1], lb2[2]], [ub2[0], ub2[1], ub2[2]] );
                }
            }

            progress_bar.update_with_percentage_preset(((idx + 1) as f64 / 6.0) * 100.0);
        }

        progress_bar.done_preset();
        Ok(Self {
            full_convex_hulls_standard_models,
            full_obbs_standard_models,
            full_bounding_spheres_standard_models,
            decomposition_convex_hulls_standard_models,
            decomposition_obbs_standard_models,
            decomposition_bounding_spheres_standard_models,
            full_convex_hulls_pseudo_models,
            full_obbs_pseudo_models,
            full_bounding_spheres_pseudo_models,
            decomposition_convex_hulls_pseudo_models,
            decomposition_obbs_pseudo_models,
            decomposition_bounding_spheres_pseudo_models,
        })
    }
}
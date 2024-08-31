use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::{ApolloLinkShapesDistanceStatisticsModule, LinkShapesDistanceStatistics};
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_robotics_core::robot_functions::robot_proximity_functions::RobotProximityFunctions;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;


impl PreprocessorModule for ApolloLinkShapesDistanceStatisticsModule {
    // type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "link_shapes_modules/link_shapes_distance_statistics_module".to_string()
    }

    fn current_version() -> String {
        "0.0.3".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let urdf_module = ApolloURDFModule::load_or_build(s, false).expect("error");
        let urdf_nalgebra_module = ApolloURDFNalgebraModule::from_urdf_module(&urdf_module);
        let chain_module = ApolloChainModule::load_or_build(s, false).expect("error");
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);
        let dof_module = ApolloDOFModule::load_or_build(s, false).expect("error");
        let bounds_module = ApolloBoundsModule::load_or_build(s, false).expect("error");

        let link_shape_modes = vec![LinkShapeMode::Full, LinkShapeMode::Decomposition];
        let link_shape_reps = vec![LinkShapeRep::ConvexHull, LinkShapeRep::OBB, LinkShapeRep::BoundingSphere];

        let mut full_convex_hulls = LinkShapesDistanceStatistics::default();
        let mut full_obbs=  LinkShapesDistanceStatistics::default();
        let mut full_bounding_spheres =  LinkShapesDistanceStatistics::default();
        let mut decomposition_convex_hulls=  LinkShapesDistanceStatistics::default();
        let mut decomposition_obbs=  LinkShapesDistanceStatistics::default();
        let mut decomposition_bounding_spheres=  LinkShapesDistanceStatistics::default();

        let num_samples = 1000_usize;

        progress_bar.set_max_increment(num_samples * 6);

        link_shape_modes.iter().for_each(|link_shape_mode| {
            link_shape_reps.iter().for_each(|link_shape_rep| {
                let num_shapes = link_shapes_module.get_shapes(*link_shape_mode, *link_shape_rep).len();

                let mut averages: Vec<Vec<f64>> = vec![vec![0.0; num_shapes ]; num_shapes ];
                let mut minimums: Vec<Vec<f64>> = vec![vec![1000000.0; num_shapes ]; num_shapes ];
                let mut maximums: Vec<Vec<f64>> = vec![vec![-1000000.0; num_shapes ]; num_shapes ];

                for i in 0..num_shapes {
                    averages[i][i] = 0.0;
                    minimums[i][i] = 0.0;
                    maximums[i][i] = 0.0;
                }

                for _ in 0..num_samples {
                    let sample = V::new(&bounds_module.sample_random_state());
                    let fk_res = RobotKinematicsFunctions::fk(&sample, &urdf_nalgebra_module, &chain_module, &dof_module);
                    let res = RobotProximityFunctions::self_contact(&link_shapes_module, &fk_res, *link_shape_mode, *link_shape_rep, None, false, 1000000.0);

                    res.outputs.iter().zip(res.shape_idxs.iter()).for_each(|(c, (i, j))| {
                        let dis = c.unwrap().dist;
                        averages[*i][*j] += dis;
                        averages[*j][*i] += dis;
                        if dis < minimums[*i][*j] {
                            minimums[*i][*j] = dis;
                            minimums[*j][*i] = dis;
                        }
                        if dis > maximums[*i][*j] {
                            maximums[*i][*j] = dis;
                            maximums[*j][*i] = dis;
                        }
                    });
                    progress_bar.increment();
                }

                averages.iter_mut().for_each(|x| x.iter_mut().for_each(|y| *y /= num_samples as f64));

                let stats = LinkShapesDistanceStatistics {
                    averages,
                    maximums,
                    minimums,
                };

                match link_shape_mode {
                    LinkShapeMode::Full => {
                        match link_shape_rep {
                            LinkShapeRep::ConvexHull => { full_convex_hulls = stats }
                            LinkShapeRep::OBB => { full_obbs = stats }
                            LinkShapeRep::BoundingSphere => { full_bounding_spheres = stats }
                        }
                    }
                    LinkShapeMode::Decomposition => {
                        match link_shape_rep {
                            LinkShapeRep::ConvexHull => { decomposition_convex_hulls = stats }
                            LinkShapeRep::OBB => { decomposition_obbs = stats }
                            LinkShapeRep::BoundingSphere => { decomposition_bounding_spheres = stats }
                        }
                    }
                }

            });
        });

        progress_bar.done_preset();
        return Ok(ApolloLinkShapesDistanceStatisticsModule {
            full_convex_hulls,
            full_obbs,
            full_bounding_spheres,
            decomposition_convex_hulls,
            decomposition_obbs,
            decomposition_bounding_spheres,
        })
    }
}
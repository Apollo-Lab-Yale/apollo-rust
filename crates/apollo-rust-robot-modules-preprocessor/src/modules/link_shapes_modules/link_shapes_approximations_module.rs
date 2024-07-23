use parry3d_f64::shape::TypedShape;
use apollo_rust_robot_modules::link_shapes_modules::link_shapes_approximations_module::{ApolloLinkShapesApproximationsModule, BoundingSphereDescriptor, OBBDescriptor};
use apollo_rust_robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::ApolloLinkShapesModule;
use apollo_rust_robotics_core::RobotPreprocessorSingleRobotDirectory;
use crate::RobotPreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;

impl RobotPreprocessorModule for ApolloLinkShapesApproximationsModule {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String {
        "link_shapes_modules/link_shapes_approximations_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &RobotPreprocessorSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);

        let mut full_obbs = vec![];
        let mut full_bounding_spheres = vec![];
        link_shapes_module.link_idx_to_full_shape_idx.iter().for_each(|i| {
            match i {
                None => {
                    full_obbs.push(None);
                }
                Some(i) => {
                    let full_obb = &link_shapes_module.full_obbs[*i];
                    let ts = full_obb.shape().as_typed_shape();
                    let h = match ts {
                        TypedShape::Cuboid(c) => {
                            c.half_extents.data.as_slice()
                        }
                        _ => { unreachable!() }
                    };
                    let offset = full_obb.offset().as_ref().unwrap();
                    let xyz = offset.0.translation.vector.as_slice();
                    let rpy = offset.0.rotation.euler_angles();
                    full_obbs.push(Some(OBBDescriptor {
                        half_extents: [h[0], h[1], h[2]],
                        offset_xyz: [xyz[0], xyz[1], xyz[2]],
                        offset_rpy: [rpy.0, rpy.1, rpy.2],
                    }));

                    let full_bounding_sphere = &link_shapes_module.full_bounding_spheres[*i];
                    let ts = full_bounding_sphere.shape().as_typed_shape();
                    let r = match ts {
                        TypedShape::Ball(b) => {
                            b.radius
                        }
                        _ => { unreachable!() }
                    };
                    let offset = full_bounding_sphere.offset().as_ref().unwrap();
                    let xyz = offset.0.translation.vector.as_slice();
                    let rpy = offset.0.rotation.euler_angles();
                    full_bounding_spheres.push(Some(BoundingSphereDescriptor {
                        radius: r,
                        offset_xyz: [xyz[0], xyz[1], xyz[2]],
                        offset_rpy: [rpy.0, rpy.1, rpy.2],
                    }));
                }
            }
        });

        let mut decomposition_obbs = vec![];
        let mut decomposition_bounding_spheres = vec![];
        link_shapes_module.link_idx_to_decomposition_shape_idxs.iter().for_each(|x| {
            let mut tmp1 = vec![];
            let mut tmp2 = vec![];
            x.iter().for_each(|i| {
                let decomposition_obb = &link_shapes_module.decomposition_obbs[*i];
                let ts = decomposition_obb.shape().as_typed_shape();
                let h = match ts {
                    TypedShape::Cuboid(c) => {
                        c.half_extents.data.as_slice()
                    }
                    _ => { unreachable!() }
                };
                let offset = decomposition_obb.offset().as_ref().unwrap();
                let xyz = offset.0.translation.vector.as_slice();
                let rpy = offset.0.rotation.euler_angles();
                tmp1.push(OBBDescriptor {
                    half_extents: [h[0], h[1], h[2]],
                    offset_xyz: [xyz[0], xyz[1], xyz[2]],
                    offset_rpy: [rpy.0, rpy.1, rpy.2],
                });

                let decomposition_bounding_sphere = &link_shapes_module.decomposition_bounding_spheres[*i];
                let ts = decomposition_bounding_sphere.shape().as_typed_shape();
                let r = match ts {
                    TypedShape::Ball(b) => {
                        b.radius
                    }
                    _ => { unreachable!() }
                };
                let offset = decomposition_bounding_sphere.offset().as_ref().unwrap();
                let xyz = offset.0.translation.vector.as_slice();
                let rpy = offset.0.rotation.euler_angles();
                tmp2.push(BoundingSphereDescriptor {
                    radius: r,
                    offset_xyz: [xyz[0], xyz[1], xyz[2]],
                    offset_rpy: [rpy.0, rpy.1, rpy.2],
                });
            });
            decomposition_obbs.push(tmp1);
            decomposition_bounding_spheres.push(tmp2);
        });

        progress_bar.done_preset();
        Ok(Self {
            full_obbs,
            full_bounding_spheres,
            decomposition_obbs,
            decomposition_bounding_spheres,
        })
    }
}
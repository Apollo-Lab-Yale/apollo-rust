use parry3d_f64::shape::ConvexPolyhedron;
use apollo_rust_proximity::offset_shape::{OffsetShape, to_offset_shape_bounding_sphere, to_offset_shape_obb};
use apollo_rust_robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules_builders::{RobotPreprocessorModule, RobotPreprocessorRobotsDirectory, RobotPreprocessorSingleRobotDirectory};
use apollo_rust_robot_modules_builders::utils::stl::load_stl_file;
use apollo_rust_robot_modules_builders::utils::trimesh::ToTriMesh;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule};
use parry3d_f64::math::Point;
use apollo_rust_file::ApolloPathBufTrait;

pub trait LinkShapesModuleBuilders {
    fn from_robot_directory(s: &RobotPreprocessorSingleRobotDirectory) -> Self;
    fn from_mesh_modules(root: &RobotPreprocessorRobotsDirectory, convex_hull_meshes_module: &ApolloConvexHullMeshesModule, convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule) -> Self;
}
impl LinkShapesModuleBuilders for ApolloLinkShapesModule {
    fn from_robot_directory(s: &RobotPreprocessorSingleRobotDirectory) -> Self {
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(s, false).expect("error");

        let root = RobotPreprocessorRobotsDirectory::new(s.robots_directory().clone());

        Self::from_mesh_modules(&root, &convex_hull_meshes_module, &convex_decomposition_meshes_module)
    }

    fn from_mesh_modules(root: &RobotPreprocessorRobotsDirectory, convex_hull_meshes_module: &ApolloConvexHullMeshesModule, convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule) -> Self {
        let mut full_convex_hulls = vec![];
        let mut full_obbs = vec![];
        let mut full_bounding_spheres = vec![];
        let mut full_shape_idx_to_link_idx = vec![];
        let mut link_idx_to_full_shape_idx = vec![];

        let mut decomposition_convex_hulls = vec![];
        let mut decomposition_obbs = vec![];
        let mut decomposition_bounding_spheres = vec![];
        let mut decomposition_shape_idx_to_link_idx_and_link_sub_idx = vec![];
        let mut link_idx_to_decomposition_shape_idxs = vec![];

        let mut count = 0;
        convex_hull_meshes_module.stl_link_mesh_relative_paths.iter().enumerate().for_each(|(link_idx, x)| {
            if let Some(path_buf) = x {
                let full_path = root.directory().clone().append_path(path_buf);
                let tm = load_stl_file(&full_path).expect("error").to_trimesh();
                let points: Vec<Point<f64>> = tm.points().iter().map(|x| Point::new(x[0], x[1], x[2])).collect();
                let cp = ConvexPolyhedron::from_convex_hull(&points).expect("error");
                let os = OffsetShape::new(cp.clone(), None);
                let obb = to_offset_shape_obb(&cp);
                let bs = to_offset_shape_bounding_sphere(&cp);

                full_convex_hulls.push(os);
                full_obbs.push(obb);
                full_bounding_spheres.push(bs);

                link_idx_to_full_shape_idx.push(Some(count));
                full_shape_idx_to_link_idx.push(link_idx);
                count += 1;
            } else {
                link_idx_to_full_shape_idx.push(None);
            }
        });

        let mut count = 0;
        convex_decomposition_meshes_module.stl_link_mesh_relative_paths.iter().enumerate().for_each(|(link_idx, x)| {
            let mut curr = vec![];
            x.iter().enumerate().for_each(|(link_sub_idx, path_buf)| {
                let full_path = root.directory().clone().append_path(path_buf);
                let tm = load_stl_file(&full_path).expect("error").to_trimesh();
                let points: Vec<Point<f64>> = tm.points().iter().map(|x| Point::new(x[0], x[1], x[2])).collect();
                let cp = ConvexPolyhedron::from_convex_hull(&points).expect("error");
                let os = OffsetShape::new(cp.clone(), None);
                let obb = to_offset_shape_obb(&cp);
                let bs = to_offset_shape_bounding_sphere(&cp);

                decomposition_convex_hulls.push(os);
                decomposition_obbs.push(obb);
                decomposition_bounding_spheres.push(bs);

                decomposition_shape_idx_to_link_idx_and_link_sub_idx.push((link_idx, link_sub_idx));
                curr.push(count);
                count += 1;
            });
            link_idx_to_decomposition_shape_idxs.push(curr);
        });

        Self {
            full_convex_hulls,
            full_obbs,
            full_bounding_spheres,
            full_shape_idx_to_link_idx,
            link_idx_to_full_shape_idx,
            decomposition_convex_hulls,
            decomposition_obbs,
            decomposition_bounding_spheres,
            decomposition_shape_idx_to_link_idx_and_link_sub_idx,
            link_idx_to_decomposition_shape_idxs,
        }
    }
}
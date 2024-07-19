use parry3d_f64::math::Point;
use parry3d_f64::shape::ConvexPolyhedron;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_proximity::offset_shape::{OffsetShape, to_offset_shape_bounding_sphere, to_offset_shape_obb};
use apollo_rust_robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::{RobotPreprocessorRobotsDirectory, RobotPreprocessorSingleRobotDirectory};

/// initialized in apollo-rust-robotics
#[derive(Clone)]
pub struct ApolloLinkShapesModule {
    pub full_convex_hulls: Vec<OffsetShape>,
    pub full_obbs: Vec<OffsetShape>,
    pub full_bounding_spheres: Vec<OffsetShape>,
    pub full_shape_idx_to_link_idx: Vec<usize>,
    pub link_idx_to_full_shape_idx: Vec<Option<usize>>,
    pub decomposition_convex_hulls: Vec<OffsetShape>,
    pub decomposition_obbs: Vec<OffsetShape>,
    pub decomposition_bounding_spheres: Vec<OffsetShape>,
    pub decomposition_shape_idx_to_link_idx_and_link_sub_idx: Vec<(usize, usize)>,
    pub link_idx_to_decomposition_shape_idxs: Vec<Vec<usize>>
}
impl ApolloLinkShapesModule {
    pub fn from_mesh_modules(s: &RobotPreprocessorSingleRobotDirectory, convex_hull_meshes_module: &ApolloConvexHullMeshesModule, convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule) -> Self {
        let root = RobotPreprocessorRobotsDirectory::new(s.robots_directory.clone());
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

    #[inline(always)]
    pub fn full_convex_hulls(&self) -> &Vec<OffsetShape> {
        &self.full_convex_hulls
    }

    #[inline(always)]
    pub fn full_obbs(&self) -> &Vec<OffsetShape> {
        &self.full_obbs
    }

    #[inline(always)]
    pub fn full_bounding_spheres(&self) -> &Vec<OffsetShape> {
        &self.full_bounding_spheres
    }

    #[inline(always)]
    pub fn decomposition_convex_hulls(&self) -> &Vec<OffsetShape> {
        &self.decomposition_convex_hulls
    }

    #[inline(always)]
    pub fn decomposition_obbs(&self) -> &Vec<OffsetShape> {
        &self.decomposition_obbs
    }

    #[inline(always)]
    pub fn decomposition_bounding_spheres(&self) -> &Vec<OffsetShape> {
        &self.decomposition_bounding_spheres
    }

    #[inline(always)]
    pub fn full_shape_idx_to_link_idx(&self) -> &Vec<usize> {
        &self.full_shape_idx_to_link_idx
    }

    #[inline(always)]
    pub fn link_idx_to_full_shape_idx(&self) -> &Vec<Option<usize>> {
        &self.link_idx_to_full_shape_idx
    }

    #[inline(always)]
    pub fn decomposition_shape_idx_to_link_idx_and_link_sub_idx(&self) -> &Vec<(usize, usize)> {
        &self.decomposition_shape_idx_to_link_idx_and_link_sub_idx
    }

    #[inline(always)]
    pub fn link_idx_to_decomposition_shape_idxs(&self) -> &Vec<Vec<usize>> {
        &self.link_idx_to_decomposition_shape_idxs
    }

    #[inline(always)]
    pub fn link_poses_to_shape_poses(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode) -> Vec<ISE3q> {
        let mut out = vec![];

        match link_shape_mode {
            LinkShapeMode::Full => {
                self.full_shape_idx_to_link_idx.iter().for_each(|x| {
                    out.push(link_poses[*x].clone());
                });
            }
            LinkShapeMode::Decomposition => {
                self.decomposition_shape_idx_to_link_idx_and_link_sub_idx.iter().for_each(|x| {
                    out.push(link_poses[x.0].clone());
                });
            }
        }

        out
    }

    #[inline(always)]
    pub fn get_shapes(&self, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> &Vec<OffsetShape> {
        match (link_shape_mode, link_shape_rep) {
            (LinkShapeMode::Full, LinkShapeRep::ConvexHull) => { &self.full_convex_hulls }
            (LinkShapeMode::Full, LinkShapeRep::OBB) => { &self.full_obbs }
            (LinkShapeMode::Full, LinkShapeRep::BoundingSphere) => { &self.full_bounding_spheres }
            (LinkShapeMode::Decomposition, LinkShapeRep::ConvexHull) => { &self.decomposition_convex_hulls }
            (LinkShapeMode::Decomposition, LinkShapeRep::OBB) => { &self.decomposition_obbs }
            (LinkShapeMode::Decomposition, LinkShapeRep::BoundingSphere) => { &self.decomposition_bounding_spheres }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkShapeMode {
    Full,
    Decomposition
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkShapeRep {
    ConvexHull,
    OBB,
    BoundingSphere
}
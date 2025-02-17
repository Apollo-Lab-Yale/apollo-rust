use parry3d_f64::math::Point;
use parry3d_f64::shape::ConvexPolyhedron;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_mesh_utils::stl::load_stl_file;
use apollo_rust_mesh_utils::trimesh::ToTriMesh;
use apollo_rust_proximity_parry::offset_shape::{OffsetShape, to_offset_shape_bounding_sphere, to_offset_shape_obb};
use apollo_rust_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

/// initialized in apollo-rust-robotics
/// The `ApolloLinkShapesModule` struct holds various link shapes for a robot, including full convex hulls, OBBs, and bounding spheres.
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
    /// Creates a new `ApolloLinkShapesModule` by loading meshes from the provided mesh modules.
    ///
    /// # Arguments
    /// - `s`: A reference to the `ResourcesSubDirectory` for the directory structure.
    /// - `convex_hull_meshes_module`: A reference to the convex hull meshes module.
    /// - `convex_decomposition_meshes_module`: A reference to the convex decomposition meshes module.
    ///
    /// # Returns
    /// A new `ApolloLinkShapesModule` instance.
    pub fn from_mesh_modules(s: &ResourcesSubDirectory, convex_hull_meshes_module: &ApolloConvexHullMeshesModule, convex_decomposition_meshes_module: &ApolloConvexDecompositionMeshesModule) -> Self {
        let root = ResourcesRootDirectory::new(s.root_directory().clone(), s.resources_type);
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
                let full_path = root.directory.clone().append_path(path_buf);
                let tm = load_stl_file(&full_path).expect(&format!("error: {:?}", full_path)).to_trimesh();
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
                let full_path = root.directory.clone().append_path(path_buf);
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

    /*
    pub fn from_environment_mesh_modules(s: &ResourcesSingleEnvironmentDirectory, convex_hull_meshes_module: &ApolloEnvironmentConvexHullMeshesModule, convex_decomposition_meshes_module: &ApolloEnvironmentConvexDecompositionMeshesModule) -> Self {
        let ss = ResourcesSingleRobotDirectory {
            robot_name: s.environment_name.clone(),
            robots_directory: s.environments_directory.clone(),
            directory: s.directory.clone(),
        };

        Self::from_robot_mesh_modules(&ss, &convex_hull_meshes_module.0, &convex_decomposition_meshes_module.0)
    }
    */

    /// Returns a reference to the full convex hull shapes.
    #[inline(always)]
    pub fn full_convex_hulls(&self) -> &Vec<OffsetShape> {
        &self.full_convex_hulls
    }

    /// Returns a reference to the full OBB shapes.
    #[inline(always)]
    pub fn full_obbs(&self) -> &Vec<OffsetShape> {
        &self.full_obbs
    }

    /// Returns a reference to the full bounding sphere shapes.
    #[inline(always)]
    pub fn full_bounding_spheres(&self) -> &Vec<OffsetShape> {
        &self.full_bounding_spheres
    }

    /// Returns a reference to the decomposition convex hull shapes.
    #[inline(always)]
    pub fn decomposition_convex_hulls(&self) -> &Vec<OffsetShape> {
        &self.decomposition_convex_hulls
    }

    /// Returns a reference to the decomposition OBB shapes.
    #[inline(always)]
    pub fn decomposition_obbs(&self) -> &Vec<OffsetShape> {
        &self.decomposition_obbs
    }

    /// Returns a reference to the decomposition bounding sphere shapes.
    #[inline(always)]
    pub fn decomposition_bounding_spheres(&self) -> &Vec<OffsetShape> {
        &self.decomposition_bounding_spheres
    }

    /// Returns a reference to the shape-to-link index mapping for full shapes.
    #[inline(always)]
    pub fn full_shape_idx_to_link_idx(&self) -> &Vec<usize> {
        &self.full_shape_idx_to_link_idx
    }

    /// Returns a reference to the link-to-shape index mapping for full shapes.
    #[inline(always)]
    pub fn link_idx_to_full_shape_idx(&self) -> &Vec<Option<usize>> {
        &self.link_idx_to_full_shape_idx
    }

    /// Returns a reference to the shape-to-link index mapping for decomposition shapes.
    #[inline(always)]
    pub fn decomposition_shape_idx_to_link_idx_and_link_sub_idx(&self) -> &Vec<(usize, usize)> {
        &self.decomposition_shape_idx_to_link_idx_and_link_sub_idx
    }

    /// Returns a reference to the link-to-shape index mapping for decomposition shapes.
    #[inline(always)]
    pub fn link_idx_to_decomposition_shape_idxs(&self) -> &Vec<Vec<usize>> {
        &self.link_idx_to_decomposition_shape_idxs
    }

    /// Maps link poses to shape poses based on the link shape mode.
    ///
    /// # Arguments
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the link poses.
    /// - `link_shape_mode`: The mode of the link shapes (full or decomposition).
    ///
    /// # Returns
    /// A vector of `ISE3q` representing the shape poses.
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

    /// Retrieves the shapes based on the link shape mode and representation.
    ///
    /// # Arguments
    /// - `link_shape_mode`: The mode of the link shapes (full or decomposition).
    /// - `link_shape_rep`: The representation of the link shapes (convex hull, OBB, or bounding sphere).
    ///
    /// # Returns
    /// A reference to a vector of `OffsetShape`.
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

    /// Retrieves the link index and subcomponent index from the shape index.
    ///
    /// # Arguments
    /// - `shape_idx`: The index of the shape.
    /// - `link_shape_mode`: A reference to the link shape mode (full or decomposition).
    ///
    /// # Returns
    /// A tuple containing the link index and the subcomponent index.
    #[inline(always)]
    pub fn get_link_idx_and_subcomponent_idx_from_shape_idx(&self, shape_idx: usize, link_shape_mode: &LinkShapeMode) -> (usize, usize) {
        return match link_shape_mode {
            LinkShapeMode::Full => {
                (self.full_shape_idx_to_link_idx[shape_idx].clone(), 0)
            }
            LinkShapeMode::Decomposition => {
                self.decomposition_shape_idx_to_link_idx_and_link_sub_idx[shape_idx].clone()
            }
        }
    }

    /// Retrieves the shape index from the link index and subcomponent index.
    ///
    /// # Arguments
    /// - `link_idx`: The index of the link.
    /// - `subcomponent_idx`: The index of the subcomponent.
    /// - `link_shape_mode`: A reference to the link shape mode (full or decomposition).
    ///
    /// # Returns
    /// An optional `usize` representing the shape index.
    pub fn get_shape_idx_from_link_idx_and_subcomponent_idx(&self, link_idx: usize, subcomponent_idx: usize, link_shape_mode: &LinkShapeMode) -> Option<usize> {
        return match link_shape_mode {
            LinkShapeMode::Full => {
                self.link_idx_to_full_shape_idx[link_idx].clone()
            }
            LinkShapeMode::Decomposition => {
                let x = self.link_idx_to_decomposition_shape_idxs[link_idx].clone();
                if x.len() == 0 { None } else {
                    Some(x[subcomponent_idx])
                }
            }
        }
    }
}

/// The `LinkShapeMode` enum represents the mode of link shapes (full or decomposition).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkShapeMode {
    Full,
    Decomposition
}

/// The `LinkShapeRep` enum represents the representation of link shapes (convex hull, OBB, or bounding sphere).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinkShapeRep {
    ConvexHull,
    OBB,
    BoundingSphere
}
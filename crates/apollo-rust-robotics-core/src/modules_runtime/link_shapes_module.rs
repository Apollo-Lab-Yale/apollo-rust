use apollo_rust_proximity::offset_shape::OffsetShape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

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
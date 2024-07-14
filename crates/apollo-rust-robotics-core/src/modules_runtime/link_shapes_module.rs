use apollo_rust_proximity::offset_shape::OffsetShape;

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
use serde::{Deserialize, Serialize};


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSkipsModule {
    pub full_convex_hulls_skips: Vec<Vec<bool>>,
    pub full_obbs_skips: Vec<Vec<bool>>,
    pub full_bounding_spheres_skips: Vec<Vec<bool>>,
    pub decomposition_convex_hulls_skips: Vec<Vec<bool>>,
    pub decomposition_obbs_skips: Vec<Vec<bool>>,
    pub decomposition_bounding_spheres_skips: Vec<Vec<bool>>
}
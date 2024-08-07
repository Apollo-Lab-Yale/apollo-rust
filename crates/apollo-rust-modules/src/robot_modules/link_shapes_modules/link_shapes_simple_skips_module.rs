use serde::{Deserialize, Serialize};


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSimpleSkipsModule {
    pub full_convex_hulls_simple_skips: Vec<Vec<bool>>,
    pub full_obbs_simple_skips: Vec<Vec<bool>>,
    pub full_bounding_spheres_simple_skips: Vec<Vec<bool>>,
    pub decomposition_convex_hulls_simple_skips: Vec<Vec<bool>>,
    pub decomposition_obbs_simple_skips: Vec<Vec<bool>>,
    pub decomposition_bounding_spheres_simple_skips: Vec<Vec<bool>>
}


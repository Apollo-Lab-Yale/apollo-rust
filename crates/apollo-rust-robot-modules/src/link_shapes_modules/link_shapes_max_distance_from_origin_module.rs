use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesMaxDistanceFromOriginModule {
    pub full_convex_hulls_maximum_distances: Vec<Option<f64>>,
    pub full_obbs_maximum_distances: Vec<Option<f64>>,
    pub full_bounding_spheres_maximum_distances: Vec<Option<f64>>,
    pub decomposition_convex_hulls_maximum_distances: Vec<Vec<f64>>,
    pub decomposition_obbs_maximum_distances: Vec<Vec<f64>>,
    pub decomposition_bounding_spheres_maximum_distances: Vec<Vec<f64>>,
}

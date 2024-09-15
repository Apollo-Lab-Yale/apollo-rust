use serde::{Deserialize, Serialize};

/// # ApolloLinkShapesSimpleSkipsModule
///
/// This struct stores boolean flags that indicate whether certain shapes should be skipped
/// during processing. The skips are represented as 2D vectors of `bool` values for both
/// full and decomposed shapes.
///
/// ## Fields:
/// - `full_convex_hulls_simple_skips`: A 2D vector of `bool` flags for full convex hulls, indicating if each shape should be skipped.
/// - `full_obbs_simple_skips`: A 2D vector of `bool` flags for full oriented bounding boxes (OBBs), indicating if each shape should be skipped.
/// - `full_bounding_spheres_simple_skips`: A 2D vector of `bool` flags for full bounding spheres, indicating if each shape should be skipped.
/// - `decomposition_convex_hulls_simple_skips`: A 2D vector of `bool` flags for decomposed convex hulls, indicating if each part should be skipped.
/// - `decomposition_obbs_simple_skips`: A 2D vector of `bool` flags for decomposed OBBs, indicating if each part should be skipped.
/// - `decomposition_bounding_spheres_simple_skips`: A 2D vector of `bool` flags for decomposed bounding spheres, indicating if each part should be skipped.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSimpleSkipsModule {
    pub full_convex_hulls_simple_skips: Vec<Vec<bool>>,
    pub full_obbs_simple_skips: Vec<Vec<bool>>,
    pub full_bounding_spheres_simple_skips: Vec<Vec<bool>>,
    pub decomposition_convex_hulls_simple_skips: Vec<Vec<bool>>,
    pub decomposition_obbs_simple_skips: Vec<Vec<bool>>,
    pub decomposition_bounding_spheres_simple_skips: Vec<Vec<bool>>
}

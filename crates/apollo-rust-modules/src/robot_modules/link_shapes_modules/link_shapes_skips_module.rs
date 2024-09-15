use serde::{Deserialize, Serialize};

/// # ApolloLinkShapesSkipsModule
///
/// This struct contains boolean flags indicating which shapes or shape components should be skipped
/// during processing. The flags are stored as 2D vectors of `bool` values for both full and decomposed shapes.
///
/// ## Fields:
/// - `full_convex_hulls_skips`: A 2D vector of `bool` flags indicating if full convex hulls should be skipped.
/// - `full_obbs_skips`: A 2D vector of `bool` flags indicating if full oriented bounding boxes (OBBs) should be skipped.
/// - `full_bounding_spheres_skips`: A 2D vector of `bool` flags indicating if full bounding spheres should be skipped.
/// - `decomposition_convex_hulls_skips`: A 2D vector of `bool` flags indicating if decomposed convex hulls should be skipped.
/// - `decomposition_obbs_skips`: A 2D vector of `bool` flags indicating if decomposed OBBs should be skipped.
/// - `decomposition_bounding_spheres_skips`: A 2D vector of `bool` flags indicating if decomposed bounding spheres should be skipped.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesSkipsModule {
    pub full_convex_hulls_skips: Vec<Vec<bool>>,
    pub full_obbs_skips: Vec<Vec<bool>>,
    pub full_bounding_spheres_skips: Vec<Vec<bool>>,
    pub decomposition_convex_hulls_skips: Vec<Vec<bool>>,
    pub decomposition_obbs_skips: Vec<Vec<bool>>,
    pub decomposition_bounding_spheres_skips: Vec<Vec<bool>>
}

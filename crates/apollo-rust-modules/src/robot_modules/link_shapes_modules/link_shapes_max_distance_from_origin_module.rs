use serde::{Deserialize, Serialize};

/// # ApolloLinkShapesMaxDistanceFromOriginModule
///
/// This struct stores the maximum distances from the origin for different shapes,
/// both in their full and decomposed forms. The full shapes use optional `f64` values,
/// while decomposed shapes are represented using 2D vectors of `f64`.
///
/// ## Fields:
/// - `full_convex_hulls_maximum_distances`: A vector of optional `f64` values representing the maximum distances for full convex hulls.
/// - `full_obbs_maximum_distances`: A vector of optional `f64` values representing the maximum distances for full OBBs (oriented bounding boxes).
/// - `full_bounding_spheres_maximum_distances`: A vector of optional `f64` values representing the maximum distances for full bounding spheres.
/// - `decomposition_convex_hulls_maximum_distances`: A 2D vector of `f64` values representing the maximum distances for decomposed convex hulls.
/// - `decomposition_obbs_maximum_distances`: A 2D vector of `f64` values representing the maximum distances for decomposed OBBs.
/// - `decomposition_bounding_spheres_maximum_distances`: A 2D vector of `f64` values representing the maximum distances for decomposed bounding spheres.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesMaxDistanceFromOriginModule {
    pub full_convex_hulls_maximum_distances: Vec<Option<f64>>,
    pub full_obbs_maximum_distances: Vec<Option<f64>>,
    pub full_bounding_spheres_maximum_distances: Vec<Option<f64>>,
    pub decomposition_convex_hulls_maximum_distances: Vec<Vec<f64>>,
    pub decomposition_obbs_maximum_distances: Vec<Vec<f64>>,
    pub decomposition_bounding_spheres_maximum_distances: Vec<Vec<f64>>,
}

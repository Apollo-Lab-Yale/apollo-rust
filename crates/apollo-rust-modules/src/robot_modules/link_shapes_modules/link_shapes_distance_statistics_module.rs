use serde::{Deserialize, Serialize};

/// # ApolloLinkShapesDistanceStatisticsModule
///
/// This struct stores distance statistics for different shape types, both in their full and decomposed forms.
/// Each field contains statistics such as averages, maximums, and minimums for various shape types.
///
/// ## Fields:
/// - `full_convex_hulls`: Distance statistics for full convex hulls.
/// - `full_obbs`: Distance statistics for full oriented bounding boxes (OBBs).
/// - `full_bounding_spheres`: Distance statistics for full bounding spheres.
/// - `decomposition_convex_hulls`: Distance statistics for decomposed convex hulls.
/// - `decomposition_obbs`: Distance statistics for decomposed OBBs.
/// - `decomposition_bounding_spheres`: Distance statistics for decomposed bounding spheres.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesDistanceStatisticsModule {
    pub full_convex_hulls: LinkShapesDistanceStatistics,
    pub full_obbs: LinkShapesDistanceStatistics,
    pub full_bounding_spheres: LinkShapesDistanceStatistics,
    pub decomposition_convex_hulls: LinkShapesDistanceStatistics,
    pub decomposition_obbs: LinkShapesDistanceStatistics,
    pub decomposition_bounding_spheres: LinkShapesDistanceStatistics,
}

/// # LinkShapesDistanceStatistics
///
/// This struct stores statistical data, including averages, maximums, and minimums for shape distances.
/// The statistics are stored in 2D vectors of floating-point numbers.
///
/// ## Fields:
/// - `averages`: A 2D vector of `f64` representing the average distances.
/// - `maximums`: A 2D vector of `f64` representing the maximum distances.
/// - `minimums`: A 2D vector of `f64` representing the minimum distances.
#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct LinkShapesDistanceStatistics {
    pub averages: Vec<Vec<f64>>,
    pub maximums: Vec<Vec<f64>>,
    pub minimums: Vec<Vec<f64>>
}

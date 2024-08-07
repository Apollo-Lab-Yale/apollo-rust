use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesDistanceStatisticsModule {
    pub full_convex_hulls: LinkShapesDistanceStatistics,
    pub full_obbs: LinkShapesDistanceStatistics,
    pub full_bounding_spheres: LinkShapesDistanceStatistics,
    pub decomposition_convex_hulls: LinkShapesDistanceStatistics,
    pub decomposition_obbs: LinkShapesDistanceStatistics,
    pub decomposition_bounding_spheres: LinkShapesDistanceStatistics,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct LinkShapesDistanceStatistics {
    pub averages: Vec<Vec<f64>>,
    pub maximums: Vec<Vec<f64>>,
    pub minimums: Vec<Vec<f64>>
}


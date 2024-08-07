use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesApproximationsModule {
    pub full_obbs: Vec<Option<OBBDescriptor>>,
    pub full_bounding_spheres: Vec<Option<BoundingSphereDescriptor>>,
    pub decomposition_obbs: Vec<Vec<OBBDescriptor>>,
    pub decomposition_bounding_spheres: Vec<Vec<BoundingSphereDescriptor>>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OBBDescriptor {
    pub half_extents: [f64; 3],
    pub offset_xyz: [f64; 3],
    pub offset_rpy: [f64; 3]
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoundingSphereDescriptor {
    pub radius: f64,
    pub offset_xyz: [f64; 3],
    pub offset_rpy: [f64; 3]
}




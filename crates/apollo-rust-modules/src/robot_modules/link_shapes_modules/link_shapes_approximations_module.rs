use serde::{Deserialize, Serialize};

/// # ApolloLinkShapesApproximationsModule
///
/// This struct contains approximations of various shapes, including oriented bounding boxes (OBBs)
/// and bounding spheres, both in full and decomposed forms. The fields store either individual
/// shape descriptors or collections of them.
///
/// ## Fields:
/// - `full_obbs`: A vector of optional `OBBDescriptor` representing full oriented bounding boxes.
/// - `full_bounding_spheres`: A vector of optional `BoundingSphereDescriptor` representing full bounding spheres.
/// - `decomposition_obbs`: A 2D vector of `OBBDescriptor` representing decomposed OBBs.
/// - `decomposition_bounding_spheres`: A 2D vector of `BoundingSphereDescriptor` representing decomposed bounding spheres.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloLinkShapesApproximationsModule {
    pub full_obbs: Vec<Option<OBBDescriptor>>,
    pub full_bounding_spheres: Vec<Option<BoundingSphereDescriptor>>,
    pub decomposition_obbs: Vec<Vec<OBBDescriptor>>,
    pub decomposition_bounding_spheres: Vec<Vec<BoundingSphereDescriptor>>,
}

/// # OBBDescriptor
///
/// This struct describes an oriented bounding box (OBB) with half extents and offset information.
///
/// ## Fields:
/// - `half_extents`: An array of three `f64` values representing the half extents of the OBB.
/// - `offset_xyz`: An array of three `f64` values representing the XYZ offset of the OBB.
/// - `offset_rpy`: An array of three `f64` values representing the roll-pitch-yaw (RPY) offset of the OBB.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct OBBDescriptor {
    pub half_extents: [f64; 3],
    pub offset_xyz: [f64; 3],
    pub offset_rpy: [f64; 3]
}

/// # BoundingSphereDescriptor
///
/// This struct describes a bounding sphere with a radius and offset information.
///
/// ## Fields:
/// - `radius`: A `f64` representing the radius of the bounding sphere.
/// - `offset_xyz`: An array of three `f64` values representing the XYZ offset of the sphere.
/// - `offset_rpy`: An array of three `f64` values representing the roll-pitch-yaw (RPY) offset of the sphere.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoundingSphereDescriptor {
    pub radius: f64,
    pub offset_xyz: [f64; 3],
    pub offset_rpy: [f64; 3]
}

use serde::{Deserialize, Serialize};
use apollo_rust_modules::robot_modules::urdf_module::*;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, ApolloIsometryMatrix3Trait, I3, I3M};
use apollo_rust_spatial::lie::se3_implicit::LieGroupISE3;
use apollo_rust_spatial::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial::matrices::M3;
use apollo_rust_spatial::vectors::V3;

/// The `ApolloURDFNalgebraModule` struct represents a URDF model in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFNalgebraModule {
    pub name: String,
    pub links: Vec<ApolloURDFLinkNalgebra>,
    pub joints: Vec<ApolloURDFJointNalgebra>,
    pub materials: Vec<ApolloURDFMaterial>
}
impl ApolloURDFNalgebraModule {
    /// Creates an `ApolloURDFNalgebraModule` from an `ApolloURDFModule`.
    ///
    /// # Arguments
    /// - `urdf_module`: A reference to the original `ApolloURDFModule`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFNalgebraModule`.
    pub fn from_urdf_module(urdf_module: &ApolloURDFModule) -> Self {
        Self {
            name: urdf_module.name.clone(),
            links: urdf_module.links.iter().map(|x| ApolloURDFLinkNalgebra::from_apollo_urdf_link(x)).collect(),
            joints: urdf_module.joints.iter().map(|x| ApolloURDFJointNalgebra::from_apollo_urdf_joint(x)).collect(),
            materials: urdf_module.materials.clone(),
        }
    }
}

/// The `ApolloURDFLinkNalgebra` struct represents a URDF link in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFLinkNalgebra {
    pub name: String,
    pub inertial: ApolloURDFInertialNalgebra,
    pub visual: Vec<ApolloURDFVisualNalgebra>,
    pub collision: Vec<ApolloURDFCollisionNalgebra>
}
impl ApolloURDFLinkNalgebra {
    /// Creates an `ApolloURDFLinkNalgebra` from an `ApolloURDFLink`.
    ///
    /// # Arguments
    /// - `apollo_urdf_link`: A reference to the original `ApolloURDFLink`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFLinkNalgebra`.
    pub fn from_apollo_urdf_link(apollo_urdf_link: &ApolloURDFLink) -> Self {
        Self {
            name: apollo_urdf_link.name.clone(),
            inertial: ApolloURDFInertialNalgebra::from_apollo_urdf_inertial(&apollo_urdf_link.inertial),
            visual: apollo_urdf_link.visual.iter().map(|x| ApolloURDFVisualNalgebra::from_apollo_urdf_visual(x)).collect(),
            collision: apollo_urdf_link.collision.iter().map(|x| ApolloURDFCollisionNalgebra::from_apollo_urdf_collision(x)).collect(),
        }
    }
}

/// The `ApolloURDFJointNalgebra` struct represents a URDF joint in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFJointNalgebra {
    pub name: String,
    pub joint_type: ApolloURDFJointType,
    pub origin: ApolloURDFPoseNalgebra,
    pub parent: ApolloURDFLinkName,
    pub child: ApolloURDFLinkName,
    pub axis: ApolloURDFAxisNalgebra,
    pub limit: ApolloURDFJointLimit,
    pub dynamics: Option<ApolloURDFDynamics>,
    pub mimic: Option<ApolloURDFMimic>,
    pub safety_controller: Option<ApolloURDFSafetyController>
}
impl ApolloURDFJointNalgebra {
    /// Creates an `ApolloURDFJointNalgebra` from an `ApolloURDFJoint`.
    ///
    /// # Arguments
    /// - `apollo_urdf_joint`: A reference to the original `ApolloURDFJoint`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFJointNalgebra`.
    pub fn from_apollo_urdf_joint(apollo_urdf_joint: &ApolloURDFJoint) -> Self {
        Self {
            name: apollo_urdf_joint.name.clone(),
            joint_type: apollo_urdf_joint.joint_type.clone(),
            origin: ApolloURDFPoseNalgebra::from_apollo_urdf_pose(&apollo_urdf_joint.origin),
            parent: apollo_urdf_joint.parent.clone(),
            child: apollo_urdf_joint.child.clone(),
            axis: ApolloURDFAxisNalgebra::from_apollo_urdf_axis(&apollo_urdf_joint.axis),
            limit: apollo_urdf_joint.limit.clone(),
            dynamics: apollo_urdf_joint.dynamics.clone(),
            mimic: apollo_urdf_joint.mimic.clone(),
            safety_controller: apollo_urdf_joint.safety_controller.clone(),
        }
    }
}

/// The `ApolloURDFInertialNalgebra` struct represents a URDF inertial in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertialNalgebra {
    pub origin: ApolloURDFPoseNalgebra,
    pub mass: ApolloURDFMass,
    pub inertia: ApolloURDFInertiaNalgebra,
}
impl ApolloURDFInertialNalgebra {
    /// Creates an `ApolloURDFInertialNalgebra` from an `ApolloURDFInertial`.
    ///
    /// # Arguments
    /// - `apollo_urdf_inertial`: A reference to the original `ApolloURDFInertial`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFInertialNalgebra`.
    pub fn from_apollo_urdf_inertial(apollo_urdf_inertial: &ApolloURDFInertial) -> Self {
        Self {
            origin: ApolloURDFPoseNalgebra::from_apollo_urdf_pose(&apollo_urdf_inertial.origin),
            mass: apollo_urdf_inertial.mass.clone(),
            inertia: ApolloURDFInertiaNalgebra::from_apollo_urdf_inertia(&apollo_urdf_inertial.inertia),
        }
    }
}

/// The `ApolloURDFInertiaNalgebra` struct represents a URDF inertia in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertiaNalgebra {
    pub inertia_matrix: M3
}
impl ApolloURDFInertiaNalgebra {
    /// Creates an `ApolloURDFInertiaNalgebra` from an `ApolloURDFInertia`.
    ///
    /// # Arguments
    /// - `apollo_urdf_inertia`: A reference to the original `ApolloURDFInertia`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFInertiaNalgebra`.
    pub fn from_apollo_urdf_inertia(apollo_urdf_inertia: &ApolloURDFInertia) -> Self {
        let inertia_matrix = M3::from_row_slice(
            &[ apollo_urdf_inertia.ixx, apollo_urdf_inertia.ixy, apollo_urdf_inertia.ixz,
                apollo_urdf_inertia.ixy, apollo_urdf_inertia.iyy, apollo_urdf_inertia.iyz,
                apollo_urdf_inertia.ixz, apollo_urdf_inertia.iyz, apollo_urdf_inertia.izz
            ]
        );
        Self {
            inertia_matrix,
        }
    }
}

/// The `ApolloURDFVisualNalgebra` struct represents a URDF visual element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFVisualNalgebra {
    pub name: Option<String>,
    pub origin: ApolloURDFPoseNalgebra,
    pub geometry: ApolloURDFGeometry,
    pub material: Option<ApolloURDFMaterial>
}
impl ApolloURDFVisualNalgebra {
    /// Creates an `ApolloURDFVisualNalgebra` from an `ApolloURDFVisual`.
    ///
    /// # Arguments
    /// - `apollo_urdf_visual`: A reference to the original `ApolloURDFVisual`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFVisualNalgebra`.
    pub fn from_apollo_urdf_visual(apollo_urdf_visual: &ApolloURDFVisual) -> Self {
        Self {
            name: apollo_urdf_visual.name.clone(),
            origin: ApolloURDFPoseNalgebra::from_apollo_urdf_pose(&apollo_urdf_visual.origin),
            geometry: apollo_urdf_visual.geometry.clone(),
            material: apollo_urdf_visual.material.clone(),
        }
    }
}

/// The `ApolloURDFCollisionNalgebra` struct represents a URDF collision element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFCollisionNalgebra {
    pub name: Option<String>,
    pub origin: ApolloURDFPoseNalgebra,
    pub geometry: ApolloURDFGeometry
}
impl ApolloURDFCollisionNalgebra {
    /// Creates an `ApolloURDFCollisionNalgebra` from an `ApolloURDFCollision`.
    ///
    /// # Arguments
    /// - `apollo_urdf_collision`: A reference to the original `ApolloURDFCollision`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFCollisionNalgebra`.
    pub fn from_apollo_urdf_collision(apollo_urdf_collision: &ApolloURDFCollision) -> Self {
        Self {
            name: apollo_urdf_collision.name.clone(),
            origin: ApolloURDFPoseNalgebra::from_apollo_urdf_pose(&apollo_urdf_collision.origin),
            geometry: apollo_urdf_collision.geometry.clone(),
        }
    }
}

/// The `ApolloURDFPoseNalgebra` struct represents a URDF pose in nalgebra form, using both quaternion and matrix representations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFPoseNalgebra {
    pub ise3q: LieGroupISE3q,
    pub ise3: LieGroupISE3
}
impl ApolloURDFPoseNalgebra {
    /// Creates an `ApolloURDFPoseNalgebra` from an `ApolloURDFPose`.
    ///
    /// # Arguments
    /// - `apollo_urdf_pose`: A reference to the original `ApolloURDFPose`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFPoseNalgebra`.
    pub fn from_apollo_urdf_pose(apollo_urdf_pose: &ApolloURDFPose) -> Self {
        let xyz = apollo_urdf_pose.xyz;
        let rpy = apollo_urdf_pose.rpy;
        Self {
            ise3q: LieGroupISE3q::new(I3::from_slices_euler_angles(&xyz, &rpy)),
            ise3: LieGroupISE3::new(I3M::from_slices_euler_angles(&xyz, &rpy)),
        }
    }
}

/// The `ApolloURDFAxisNalgebra` struct represents a URDF axis in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFAxisNalgebra {
    pub axis: V3
}
impl ApolloURDFAxisNalgebra {
    /// Creates an `ApolloURDFAxisNalgebra` from an `ApolloURDFAxis`.
    ///
    /// # Arguments
    /// - `apollo_urdf_axis`: A reference to the original `ApolloURDFAxis`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFAxisNalgebra`.
    pub fn from_apollo_urdf_axis(apollo_urdf_axis: &ApolloURDFAxis) -> Self {
        Self {
            axis: V3::from_column_slice(&apollo_urdf_axis.xyz),
        }
    }
}
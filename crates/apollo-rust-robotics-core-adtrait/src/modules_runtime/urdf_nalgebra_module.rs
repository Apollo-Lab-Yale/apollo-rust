use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_modules::robot_modules::urdf_module::*;
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait, ApolloIsometryMatrix3Trait, I3, I3M};
use apollo_rust_spatial_adtrait::lie::se3_implicit::LieGroupISE3;
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::LieGroupISE3q;
use apollo_rust_spatial_adtrait::matrices::{ApolloMatrix3ADTrait, M3};
use apollo_rust_spatial_adtrait::vectors::{ApolloVector3ADTrait, V3};

/// The `ApolloURDFNalgebraModule` struct represents a URDF model in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFNalgebraModule<A: AD> {
    pub name: String,
    #[serde(deserialize_with = "Vec::<ApolloURDFLinkNalgebra::<A>>::deserialize")]
    pub links: Vec<ApolloURDFLinkNalgebra<A>>,
    #[serde(deserialize_with = "Vec::<ApolloURDFJointNalgebra::<A>>::deserialize")]
    pub joints: Vec<ApolloURDFJointNalgebra<A>>,
    pub materials: Vec<ApolloURDFMaterial>
}
impl<A: AD> ApolloURDFNalgebraModule<A> {
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
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFNalgebraModule<A2> {
        ApolloURDFNalgebraModule {
            name: self.name.clone(),
            links: self.links.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            joints: self.joints.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            materials: self.materials.clone(),
        }
    }
}

/// The `ApolloURDFLinkNalgebra` struct represents a URDF link in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFLinkNalgebra<A: AD> {
    pub name: String,
    #[serde(deserialize_with = "ApolloURDFInertialNalgebra::<A>::deserialize")]
    pub inertial: ApolloURDFInertialNalgebra<A>,
    #[serde(deserialize_with = "Vec::<ApolloURDFVisualNalgebra::<A>>::deserialize")]
    pub visual: Vec<ApolloURDFVisualNalgebra<A>>,
    #[serde(deserialize_with = "Vec::<ApolloURDFCollisionNalgebra::<A>>::deserialize")]
    pub collision: Vec<ApolloURDFCollisionNalgebra<A>>
}
impl<A: AD> ApolloURDFLinkNalgebra<A> {
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
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFLinkNalgebra<A2> {
        ApolloURDFLinkNalgebra {
            name: self.name.clone(),
            inertial: self.inertial.to_other_ad_type::<A2>(),
            visual: self.visual.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            collision: self.collision.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
        }
    }
}

/// The `ApolloURDFJointNalgebra` struct represents a URDF joint in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFJointNalgebra<A: AD> {
    pub name: String,
    pub joint_type: ApolloURDFJointType,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebra::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebra<A>,
    pub parent: ApolloURDFLinkName,
    pub child: ApolloURDFLinkName,
    #[serde(deserialize_with = "ApolloURDFAxisNalgebra::<A>::deserialize")]
    pub axis: ApolloURDFAxisNalgebra<A>,
    pub limit: ApolloURDFJointLimit,
    pub dynamics: Option<ApolloURDFDynamics>,
    pub mimic: Option<ApolloURDFMimic>,
    pub safety_controller: Option<ApolloURDFSafetyController>
}
impl<A: AD> ApolloURDFJointNalgebra<A> {
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
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFJointNalgebra<A2> {
        ApolloURDFJointNalgebra {
            name: self.name.clone(),
            joint_type: self.joint_type.clone(),
            origin: self.origin.to_other_ad_type::<A2>(),
            parent: self.parent.clone(),
            child: self.child.clone(),
            axis: self.axis.to_other_ad_type::<A2>(),
            limit: self.limit.clone(),
            dynamics: self.dynamics.clone(),
            mimic: self.mimic.clone(),
            safety_controller: self.safety_controller.clone(),
        }
    }
}

/// The `ApolloURDFInertialNalgebra` struct represents a URDF inertial in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertialNalgebra<A: AD> {
    #[serde(deserialize_with = "ApolloURDFPoseNalgebra::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebra<A>,
    pub mass: ApolloURDFMass,
    #[serde(deserialize_with = "ApolloURDFInertiaNalgebra::<A>::deserialize")]
    pub inertia: ApolloURDFInertiaNalgebra<A>,
}
impl<A: AD> ApolloURDFInertialNalgebra<A> {
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
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFInertialNalgebra<A2> {
        ApolloURDFInertialNalgebra {
            origin: self.origin.to_other_ad_type::<A2>(),
            mass: self.mass.clone(),
            inertia: self.inertia.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFInertiaNalgebra` struct represents a URDF inertia in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertiaNalgebra<A: AD> {
    #[serde(deserialize_with = "M3::<A>::deserialize")]
    pub inertia_matrix: M3<A>
}
impl<A: AD> ApolloURDFInertiaNalgebra<A> {
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
        )
            .to_other_ad_type::<A>();
        Self {
            inertia_matrix,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFInertiaNalgebra<A2> {
        ApolloURDFInertiaNalgebra {
            inertia_matrix: self.inertia_matrix.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFVisualNalgebra` struct represents a URDF visual element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFVisualNalgebra<A: AD> {
    pub name: Option<String>,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebra::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebra<A>,
    pub geometry: ApolloURDFGeometry,
    pub material: Option<ApolloURDFMaterial>
}
impl<A: AD> ApolloURDFVisualNalgebra<A> {
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
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFVisualNalgebra<A2> {
        ApolloURDFVisualNalgebra {
            name: self.name.clone(),
            origin: self.origin.to_other_ad_type::<A2>(),
            geometry: self.geometry.clone(),
            material: self.material.clone(),
        }
    }
}

/// The `ApolloURDFCollisionNalgebra` struct represents a URDF collision element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFCollisionNalgebra<A: AD> {
    pub name: Option<String>,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebra::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebra<A>,
    pub geometry: ApolloURDFGeometry
}
impl<A: AD> ApolloURDFCollisionNalgebra<A> {
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

    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFCollisionNalgebra<A2> {
        ApolloURDFCollisionNalgebra {
            name: self.name.clone(),
            origin: self.origin.to_other_ad_type::<A2>(),
            geometry: self.geometry.clone(),
        }
    }
}

/// The `ApolloURDFPoseNalgebra` struct represents a URDF pose in nalgebra form, using both quaternion and matrix representations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFPoseNalgebra<A: AD> {
    #[serde(deserialize_with = "LieGroupISE3q::<A>::deserialize")]
    pub ise3q: LieGroupISE3q<A>,
    #[serde(deserialize_with = "LieGroupISE3::<A>::deserialize")]
    pub ise3: LieGroupISE3<A>
}
impl<A: AD> ApolloURDFPoseNalgebra<A> {
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
            ise3q: LieGroupISE3q::new(I3::from_slices_euler_angles(&xyz, &rpy).to_other_ad_type::<A>()),
            ise3: LieGroupISE3::new(I3M::from_slices_euler_angles(&xyz, &rpy).to_other_ad_type::<A>()),
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFPoseNalgebra<A2> {
        ApolloURDFPoseNalgebra {
            ise3q: self.ise3q.to_other_ad_type::<A2>(),
            ise3: self.ise3.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFAxisNalgebra` struct represents a URDF axis in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFAxisNalgebra<A: AD> {
    #[serde(deserialize_with = "V3::<A>::deserialize")]
    pub axis: V3<A>
}
impl<A: AD> ApolloURDFAxisNalgebra<A> {
    /// Creates an `ApolloURDFAxisNalgebra` from an `ApolloURDFAxis`.
    ///
    /// # Arguments
    /// - `apollo_urdf_axis`: A reference to the original `ApolloURDFAxis`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFAxisNalgebra`.
    pub fn from_apollo_urdf_axis(apollo_urdf_axis: &ApolloURDFAxis) -> Self {
        Self {
            axis: V3::from_column_slice(&apollo_urdf_axis.xyz).to_other_ad_type::<A>(),
        }
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFAxisNalgebra<A2> {
        ApolloURDFAxisNalgebra {
            axis: self.axis.to_other_ad_type::<A2>(),
        }
    }
}
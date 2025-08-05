use ad_trait::AD;
use serde::{Deserialize, Serialize};
use apollo_rust_modules::robot_modules::urdf_module::*;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::{ApolloURDFAxisNalgebra, ApolloURDFCollisionNalgebra, ApolloURDFInertialNalgebra, ApolloURDFInertiaNalgebra, ApolloURDFJointNalgebra, ApolloURDFLinkNalgebra, ApolloURDFNalgebraModule, ApolloURDFPoseNalgebra, ApolloURDFVisualNalgebra};
use apollo_rust_spatial::lie::se3_implicit::ISE3;
use apollo_rust_spatial::lie::se3_implicit_quaternion::{ISE3q};
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait as ApolloIsometry3TraitADTrait, ApolloIsometryMatrix3Trait as ApolloIsometryMatrix3TraitADTrait, I3 as I3ADTrait, I3M as I3MADTrait};
use apollo_rust_spatial_adtrait::lie::se3_implicit::{LieGroupISE3 as LieGroupISE3ADTrait};
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::{LieGroupISE3q as LieGroupISE3qADTrait};
use apollo_rust_spatial_adtrait::matrices::{ApolloMatrix3ADTrait as ApolloMatrix3ADTraitADTrait, M3 as M3ADTrait};
use apollo_rust_spatial_adtrait::vectors::{ApolloVector3ADTrait as ApolloVector3ADTraitADTrait, V3 as V3ADTrait};

/// The `ApolloURDFNalgebraModule` struct represents a URDF model in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFNalgebraModuleADTrait<A: AD> {
    pub name: String,
    #[serde(deserialize_with = "Vec::<ApolloURDFLinkNalgebraADTrait::<A>>::deserialize")]
    pub links: Vec<ApolloURDFLinkNalgebraADTrait<A>>,
    #[serde(deserialize_with = "Vec::<ApolloURDFJointNalgebraADTrait::<A>>::deserialize")]
    pub joints: Vec<ApolloURDFJointNalgebraADTrait<A>>,
    pub materials: Vec<ApolloURDFMaterial>
}
impl<A: AD> ApolloURDFNalgebraModuleADTrait<A> {
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
            links: urdf_module.links.iter().map(|x| ApolloURDFLinkNalgebraADTrait::from_apollo_urdf_link(x)).collect(),
            joints: urdf_module.joints.iter().map(|x| ApolloURDFJointNalgebraADTrait::from_apollo_urdf_joint(x)).collect(),
            materials: urdf_module.materials.clone(),
        }
    }
    pub fn to_urdf_module(&self) -> ApolloURDFNalgebraModule {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFNalgebraModule {
            name: tmp.name,
            links: tmp.links.iter().map(|x| x.to_apollo_urdf_link()).collect(),
            joints: tmp.joints.iter().map(|x| x.to_apollo_urdf_joint()).collect(),
            materials: tmp.materials,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFNalgebraModuleADTrait<A2> {
        ApolloURDFNalgebraModuleADTrait {
            name: self.name.clone(),
            links: self.links.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            joints: self.joints.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            materials: self.materials.clone(),
        }
    }
}

/// The `ApolloURDFLinkNalgebra` struct represents a URDF link in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFLinkNalgebraADTrait<A: AD> {
    pub name: String,
    #[serde(deserialize_with = "ApolloURDFInertialNalgebraADTrait::<A>::deserialize")]
    pub inertial: ApolloURDFInertialNalgebraADTrait<A>,
    #[serde(deserialize_with = "Vec::<ApolloURDFVisualNalgebraADTrait::<A>>::deserialize")]
    pub visual: Vec<ApolloURDFVisualNalgebraADTrait<A>>,
    #[serde(deserialize_with = "Vec::<ApolloURDFCollisionNalgebraADTrait::<A>>::deserialize")]
    pub collision: Vec<ApolloURDFCollisionNalgebraADTrait<A>>
}
impl<A: AD> ApolloURDFLinkNalgebraADTrait<A> {
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
            inertial: ApolloURDFInertialNalgebraADTrait::from_apollo_urdf_inertial(&apollo_urdf_link.inertial),
            visual: apollo_urdf_link.visual.iter().map(|x| ApolloURDFVisualNalgebraADTrait::from_apollo_urdf_visual(x)).collect(),
            collision: apollo_urdf_link.collision.iter().map(|x| ApolloURDFCollisionNalgebraADTrait::from_apollo_urdf_collision(x)).collect(),
        }
    }
    pub fn to_apollo_urdf_link(&self) -> ApolloURDFLinkNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFLinkNalgebra {
            name: tmp.name,
            inertial: tmp.inertial.to_apollo_urdf_inertial(),
            visual: tmp.visual.iter().map(|x| x.to_apollo_urdf_visual()).collect(),
            collision: tmp.collision.iter().map(|x| x.to_apollo_urdf_collision()).collect(),
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFLinkNalgebraADTrait<A2> {
        ApolloURDFLinkNalgebraADTrait {
            name: self.name.clone(),
            inertial: self.inertial.to_other_ad_type::<A2>(),
            visual: self.visual.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            collision: self.collision.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
        }
    }
}

/// The `ApolloURDFJointNalgebra` struct represents a URDF joint in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFJointNalgebraADTrait<A: AD> {
    pub name: String,
    pub joint_type: ApolloURDFJointType,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebraADTrait::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebraADTrait<A>,
    pub parent: ApolloURDFLinkName,
    pub child: ApolloURDFLinkName,
    #[serde(deserialize_with = "ApolloURDFAxisNalgebraADTrait::<A>::deserialize")]
    pub axis: ApolloURDFAxisNalgebraADTrait<A>,
    pub limit: ApolloURDFJointLimit,
    pub dynamics: Option<ApolloURDFDynamics>,
    pub mimic: Option<ApolloURDFMimic>,
    pub safety_controller: Option<ApolloURDFSafetyController>
}
impl<A: AD> ApolloURDFJointNalgebraADTrait<A> {
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
            origin: ApolloURDFPoseNalgebraADTrait::from_apollo_urdf_pose(&apollo_urdf_joint.origin),
            parent: apollo_urdf_joint.parent.clone(),
            child: apollo_urdf_joint.child.clone(),
            axis: ApolloURDFAxisNalgebraADTrait::from_apollo_urdf_axis(&apollo_urdf_joint.axis),
            limit: apollo_urdf_joint.limit.clone(),
            dynamics: apollo_urdf_joint.dynamics.clone(),
            mimic: apollo_urdf_joint.mimic.clone(),
            safety_controller: apollo_urdf_joint.safety_controller.clone(),
        }
    }
    pub fn to_apollo_urdf_joint(&self) -> ApolloURDFJointNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFJointNalgebra {
            name: tmp.name,
            joint_type: tmp.joint_type,
            origin: tmp.origin.to_apollo_urdf_pose(),
            parent: tmp.parent,
            child: tmp.child,
            axis: tmp.axis.to_apollo_urdf_axis(),
            limit: tmp.limit,
            dynamics: tmp.dynamics,
            mimic: tmp.mimic,
            safety_controller: tmp.safety_controller,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFJointNalgebraADTrait<A2> {
        ApolloURDFJointNalgebraADTrait {
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
pub struct ApolloURDFInertialNalgebraADTrait<A: AD> {
    #[serde(deserialize_with = "ApolloURDFPoseNalgebraADTrait::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebraADTrait<A>,
    pub mass: ApolloURDFMass,
    #[serde(deserialize_with = "ApolloURDFInertiaNalgebraADTrait::<A>::deserialize")]
    pub inertia: ApolloURDFInertiaNalgebraADTrait<A>,
}
impl<A: AD> ApolloURDFInertialNalgebraADTrait<A> {
    /// Creates an `ApolloURDFInertialNalgebra` from an `ApolloURDFInertial`.
    ///
    /// # Arguments
    /// - `apollo_urdf_inertial`: A reference to the original `ApolloURDFInertial`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFInertialNalgebra`.
    pub fn from_apollo_urdf_inertial(apollo_urdf_inertial: &ApolloURDFInertial) -> Self {
        Self {
            origin: ApolloURDFPoseNalgebraADTrait::from_apollo_urdf_pose(&apollo_urdf_inertial.origin),
            mass: apollo_urdf_inertial.mass.clone(),
            inertia: ApolloURDFInertiaNalgebraADTrait::from_apollo_urdf_inertia(&apollo_urdf_inertial.inertia),
        }
    }
    pub fn to_apollo_urdf_inertial(&self) -> ApolloURDFInertialNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFInertialNalgebra {
            origin: self.origin.to_apollo_urdf_pose(),
            mass: tmp.mass,
            inertia: self.inertia.to_apollo_urdf_inertia(),
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFInertialNalgebraADTrait<A2> {
        ApolloURDFInertialNalgebraADTrait {
            origin: self.origin.to_other_ad_type::<A2>(),
            mass: self.mass.clone(),
            inertia: self.inertia.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFInertiaNalgebra` struct represents a URDF inertia in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertiaNalgebraADTrait<A: AD> {
    #[serde(deserialize_with = "M3ADTrait::<A>::deserialize")]
    pub inertia_matrix: M3ADTrait<A>
}
impl<A: AD> ApolloURDFInertiaNalgebraADTrait<A> {
    /// Creates an `ApolloURDFInertiaNalgebra` from an `ApolloURDFInertia`.
    ///
    /// # Arguments
    /// - `apollo_urdf_inertia`: A reference to the original `ApolloURDFInertia`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFInertiaNalgebra`.
    pub fn from_apollo_urdf_inertia(apollo_urdf_inertia: &ApolloURDFInertia) -> Self {
        let inertia_matrix = M3ADTrait::from_row_slice(
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
    pub fn to_apollo_urdf_inertia(&self) -> ApolloURDFInertiaNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFInertiaNalgebra {
            inertia_matrix: tmp.inertia_matrix,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFInertiaNalgebraADTrait<A2> {
        ApolloURDFInertiaNalgebraADTrait {
            inertia_matrix: self.inertia_matrix.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFVisualNalgebra` struct represents a URDF visual element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFVisualNalgebraADTrait<A: AD> {
    pub name: Option<String>,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebraADTrait::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebraADTrait<A>,
    pub geometry: ApolloURDFGeometry,
    pub material: Option<ApolloURDFMaterial>
}
impl<A: AD> ApolloURDFVisualNalgebraADTrait<A> {
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
            origin: ApolloURDFPoseNalgebraADTrait::from_apollo_urdf_pose(&apollo_urdf_visual.origin),
            geometry: apollo_urdf_visual.geometry.clone(),
            material: apollo_urdf_visual.material.clone(),
        }
    }
    pub fn to_apollo_urdf_visual(&self) -> ApolloURDFVisualNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFVisualNalgebra {
            name: tmp.name.clone(),
            origin: self.origin.to_apollo_urdf_pose(),
            geometry: tmp.geometry,
            material: tmp.material,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFVisualNalgebraADTrait<A2> {
        ApolloURDFVisualNalgebraADTrait {
            name: self.name.clone(),
            origin: self.origin.to_other_ad_type::<A2>(),
            geometry: self.geometry.clone(),
            material: self.material.clone(),
        }
    }
}

/// The `ApolloURDFCollisionNalgebra` struct represents a URDF collision element in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFCollisionNalgebraADTrait<A: AD> {
    pub name: Option<String>,
    #[serde(deserialize_with = "ApolloURDFPoseNalgebraADTrait::<A>::deserialize")]
    pub origin: ApolloURDFPoseNalgebraADTrait<A>,
    pub geometry: ApolloURDFGeometry
}
impl<A: AD> ApolloURDFCollisionNalgebraADTrait<A> {
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
            origin: ApolloURDFPoseNalgebraADTrait::from_apollo_urdf_pose(&apollo_urdf_collision.origin),
            geometry: apollo_urdf_collision.geometry.clone(),
        }
    }
    pub fn to_apollo_urdf_collision(&self) -> ApolloURDFCollisionNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFCollisionNalgebra {
            name: tmp.name.clone(),
            origin: self.origin.to_apollo_urdf_pose(),
            geometry: tmp.geometry,
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFCollisionNalgebraADTrait<A2> {
        ApolloURDFCollisionNalgebraADTrait {
            name: self.name.clone(),
            origin: self.origin.to_other_ad_type::<A2>(),
            geometry: self.geometry.clone(),
        }
    }
}

/// The `ApolloURDFPoseNalgebra` struct represents a URDF pose in nalgebra form, using both quaternion and matrix representations.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFPoseNalgebraADTrait<A: AD> {
    #[serde(deserialize_with = "LieGroupISE3qADTrait::<A>::deserialize")]
    pub ise3q: LieGroupISE3qADTrait<A>,
    #[serde(deserialize_with = "LieGroupISE3ADTrait::<A>::deserialize")]
    pub ise3: LieGroupISE3ADTrait<A>
}
impl<A: AD> ApolloURDFPoseNalgebraADTrait<A> {
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
        let xyz_ad: [A; 3] = [
            A::constant(xyz[0]),
            A::constant(xyz[1]),
            A::constant(xyz[2])
        ];
        let rpy_ad: [A; 3] = [
            A::constant(rpy[0]),
            A::constant(rpy[1]),
            A::constant(rpy[2])
        ];

        Self {
            ise3q: LieGroupISE3qADTrait::new(I3ADTrait::from_slices_euler_angles(&xyz_ad, &rpy_ad)),
            ise3: LieGroupISE3ADTrait::new(I3MADTrait::from_slices_euler_angles(&xyz_ad, &rpy_ad)),
        }
    }
    pub fn to_apollo_urdf_pose(&self) -> ApolloURDFPoseNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        let ise3q = ISE3q::new(tmp.ise3q.0);
        let ise3 = ISE3::new(tmp.ise3.0);

        ApolloURDFPoseNalgebra {
            ise3q,
            ise3
        }
    }
    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFPoseNalgebraADTrait<A2> {
        ApolloURDFPoseNalgebraADTrait {
            ise3q: self.ise3q.to_other_ad_type::<A2>(),
            ise3: self.ise3.to_other_ad_type::<A2>(),
        }
    }
}

/// The `ApolloURDFAxisNalgebra` struct represents a URDF axis in nalgebra form.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFAxisNalgebraADTrait<A: AD> {
    #[serde(deserialize_with = "V3ADTrait::<A>::deserialize")]
    pub axis: V3ADTrait<A>
}
impl<A: AD> ApolloURDFAxisNalgebraADTrait<A> {
    /// Creates an `ApolloURDFAxisNalgebra` from an `ApolloURDFAxis`.
    ///
    /// # Arguments
    /// - `apollo_urdf_axis`: A reference to the original `ApolloURDFAxis`.
    ///
    /// # Returns
    /// A new instance of `ApolloURDFAxisNalgebra`.
    pub fn from_apollo_urdf_axis(apollo_urdf_axis: &ApolloURDFAxis) -> Self {
        Self {
            axis: V3ADTrait::from_column_slice(&apollo_urdf_axis.xyz).to_other_ad_type::<A>(),
        }
    }
    pub fn to_apollo_urdf_axis(&self) -> ApolloURDFAxisNalgebra {
        let tmp = self.to_other_ad_type::<f64>();

        ApolloURDFAxisNalgebra {
            axis: tmp.axis,
        }
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloURDFAxisNalgebraADTrait<A2> {
        ApolloURDFAxisNalgebraADTrait {
            axis: self.axis.to_other_ad_type::<A2>(),
        }
    }
}
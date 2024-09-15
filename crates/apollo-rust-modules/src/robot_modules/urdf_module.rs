use serde::{Deserialize, Serialize};
use urdf_rs::*;

/// Struct representing a URDF module that holds information about the robot's links, joints, and materials.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFModule {
    /// The name of the URDF model.
    pub name: String,

    /// A vector of URDF links in the model.
    pub links: Vec<ApolloURDFLink>,

    /// A vector of URDF joints in the model.
    pub joints: Vec<ApolloURDFJoint>,

    /// A vector of materials used in the URDF model.
    pub materials: Vec<ApolloURDFMaterial>,
}

impl ApolloURDFModule {
    /// Checks whether the link at the given index has a mesh.
    ///
    /// - `link_idx`: Index of the link to check.
    ///
    /// Returns `true` if the link has a visual mesh, otherwise `false`.
    pub fn link_has_mesh(&self, link_idx: usize) -> bool {
        return !self.links[link_idx].visual.is_empty()
    }
}

/// Struct representing a URDF link with inertial, visual, and collision properties.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFLink {
    /// The name of the link.
    pub name: String,

    /// Inertial properties of the link.
    pub inertial: ApolloURDFInertial,

    /// Visual representations of the link.
    pub visual: Vec<ApolloURDFVisual>,

    /// Collision geometries of the link.
    pub collision: Vec<ApolloURDFCollision>,
}

impl ApolloURDFLink {
    /// Converts a URDF `Link` to an `ApolloURDFLink`.
    ///
    /// - `link`: Reference to the URDF `Link` to be converted.
    ///
    /// Returns an instance of `ApolloURDFLink`.
    pub fn from_link(link: &Link) -> Self {
        Self {
            name: link.name.clone(),
            inertial: ApolloURDFInertial::from_inertial(&link.inertial),
            visual: link.visual.iter().map(|x| ApolloURDFVisual::from_visual(x)).collect(),
            collision: link.collision.iter().map(|x| ApolloURDFCollision::from_collision(x)).collect(),
        }
    }
}

/// Struct representing a URDF joint with associated properties like type, origin, parent, and child links.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFJoint {
    /// The name of the joint.
    pub name: String,

    /// The type of the joint.
    pub joint_type: ApolloURDFJointType,

    /// The pose of the joint in the parent frame.
    pub origin: ApolloURDFPose,

    /// The name of the parent link.
    pub parent: ApolloURDFLinkName,

    /// The name of the child link.
    pub child: ApolloURDFLinkName,

    /// The axis of the joint.
    pub axis: ApolloURDFAxis,

    /// The limits of the joint.
    pub limit: ApolloURDFJointLimit,

    /// Dynamics associated with the joint.
    pub dynamics: Option<ApolloURDFDynamics>,

    /// Mimic joint properties if applicable.
    pub mimic: Option<ApolloURDFMimic>,

    /// Safety controller associated with the joint.
    pub safety_controller: Option<ApolloURDFSafetyController>,
}

impl ApolloURDFJoint {
    /// Converts a URDF `Joint` to an `ApolloURDFJoint`.
    ///
    /// - `joint`: Reference to the URDF `Joint` to be converted.
    ///
    /// Returns an instance of `ApolloURDFJoint`.
    pub fn from_joint(joint: &Joint) -> Self {
        Self {
            name: joint.name.clone(),
            joint_type: ApolloURDFJointType::from_joint_type(&joint.joint_type),
            origin: ApolloURDFPose::from_pose(&joint.origin),
            parent: ApolloURDFLinkName::from_link_name(&joint.parent),
            child: ApolloURDFLinkName::from_link_name(&joint.child),
            axis: ApolloURDFAxis::from_axis(&joint.axis),
            limit: ApolloURDFJointLimit::from_joint_limit(&joint.limit),
            dynamics: match &joint.dynamics {
                None => None,
                Some(s) => Some(ApolloURDFDynamics::from_dynamics(s)),
            },
            mimic: match &joint.mimic {
                None => None,
                Some(s) => Some(ApolloURDFMimic::from_mimic(s)),
            },
            safety_controller: match &joint.safety_controller {
                None => None,
                Some(s) => Some(ApolloURDFSafetyController::from_safety_controller(s)),
            },
        }
    }
}

/// Struct representing a material in URDF, with optional color and texture.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFMaterial {
    /// The name of the material.
    pub name: String,

    /// The color of the material.
    pub color: Option<ApolloURDFColor>,

    /// The texture of the material.
    pub texture: Option<ApolloURDFTexture>,
}

impl ApolloURDFMaterial {
    /// Converts a URDF `Material` to an `ApolloURDFMaterial`.
    ///
    /// - `material`: Reference to the URDF `Material` to be converted.
    ///
    /// Returns an instance of `ApolloURDFMaterial`.
    pub fn from_material(material: &Material) -> Self {
        Self {
            name: material.name.clone(),
            color: material.color.as_ref().map(ApolloURDFColor::from_color),
            texture: material.texture.as_ref().map(ApolloURDFTexture::from_texture),
        }
    }
}

/// Struct representing the inertial properties of a URDF link.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFInertial {
    /// The origin of the inertial properties relative to the link frame.
    pub origin: ApolloURDFPose,

    /// The mass of the link.
    pub mass: ApolloURDFMass,

    /// The inertia matrix of the link.
    pub inertia: ApolloURDFInertia,
}

impl ApolloURDFInertial {
    /// Converts a URDF `Inertial` to an `ApolloURDFInertial`.
    ///
    /// - `inertial`: Reference to the URDF `Inertial` to be converted.
    ///
    /// Returns an instance of `ApolloURDFInertial`.
    pub fn from_inertial(inertial: &Inertial) -> Self {
        Self {
            origin: ApolloURDFPose::from_pose(&inertial.origin),
            mass: ApolloURDFMass::from_mass(&inertial.mass),
            inertia: ApolloURDFInertia::from_inertia(&inertial.inertia),
        }
    }
}

/// Struct representing the visual properties of a URDF link, including geometry and material.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFVisual {
    /// Optional name of the visual element.
    pub name: Option<String>,

    /// Pose of the visual element relative to the link frame.
    pub origin: ApolloURDFPose,

    /// Geometry of the visual element.
    pub geometry: ApolloURDFGeometry,

    /// Optional material of the visual element.
    pub material: Option<ApolloURDFMaterial>,
}

impl ApolloURDFVisual {
    /// Converts a URDF `Visual` to an `ApolloURDFVisual`.
    ///
    /// - `visual`: Reference to the URDF `Visual` to be converted.
    ///
    /// Returns an instance of `ApolloURDFVisual`.
    pub fn from_visual(visual: &Visual) -> Self {
        Self {
            name: visual.name.clone(),
            origin: ApolloURDFPose::from_pose(&visual.origin),
            geometry: ApolloURDFGeometry::from_geometry(&visual.geometry),
            material: visual.material.as_ref().map(ApolloURDFMaterial::from_material),
        }
    }
}

/// Struct representing a collision element of a URDF link.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFCollision {
    /// Optional name of the collision element.
    pub name: Option<String>,

    /// Pose of the collision element relative to the link frame.
    pub origin: ApolloURDFPose,

    /// Geometry of the collision element.
    pub geometry: ApolloURDFGeometry,
}

impl ApolloURDFCollision {
    /// Converts a URDF `Collision` to an `ApolloURDFCollision`.
    ///
    /// - `collision`: Reference to the URDF `Collision` to be converted.
    ///
    /// Returns an instance of `ApolloURDFCollision`.
    pub fn from_collision(collision: &Collision) -> Self {
        Self {
            name: collision.name.clone(),
            origin: ApolloURDFPose::from_pose(&collision.origin),
            geometry: ApolloURDFGeometry::from_geometry(&collision.geometry),
        }
    }
}

/// Enum representing the types of joints in URDF.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub enum ApolloURDFJointType {
    #[default]
    Revolute,
    Continuous,
    Prismatic,
    Fixed,
    Floating,
    Planar,
    Spherical,
}

impl ApolloURDFJointType {
    /// Converts a URDF `JointType` to an `ApolloURDFJointType`.
    ///
    /// - `joint_type`: Reference to the URDF `JointType` to be converted.
    ///
    /// Returns an instance of `ApolloURDFJointType`.
    pub fn from_joint_type(joint_type: &JointType) -> Self {
        match joint_type {
            JointType::Revolute => Self::Revolute,
            JointType::Continuous => Self::Continuous,
            JointType::Prismatic => Self::Prismatic,
            JointType::Fixed => Self::Fixed,
            JointType::Floating => Self::Floating,
            JointType::Planar => Self::Planar,
            JointType::Spherical => Self::Spherical,
        }
    }
}

/// Struct representing a pose in URDF, defined by position (xyz) and orientation (rpy).
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFPose {
    /// Position of the pose in the format [x, y, z].
    pub xyz: [f64; 3],

    /// Orientation of the pose in roll-pitch-yaw format [roll, pitch, yaw].
    pub rpy: [f64; 3],
}

impl ApolloURDFPose {
    /// Converts a URDF `Pose` to an `ApolloURDFPose`.
    ///
    /// - `pose`: Reference to the URDF `Pose` to be converted.
    ///
    /// Returns an instance of `ApolloURDFPose`.
    pub fn from_pose(pose: &Pose) -> Self {
        Self {
            xyz: pose.xyz.0,
            rpy: pose.rpy.0,
        }
    }

    /// Creates a new `ApolloURDFPose` with specified position and orientation.
    ///
    /// - `xyz`: Position [x, y, z].
    /// - `rpy`: Orientation [roll, pitch, yaw].
    pub fn new(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        Self { xyz, rpy }
    }
}

/// Struct representing the name of a link in URDF.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFLinkName {
    /// The name of the link.
    pub link: String,
}

impl ApolloURDFLinkName {
    /// Converts a URDF `LinkName` to an `ApolloURDFLinkName`.
    ///
    /// - `link_name`: Reference to the URDF `LinkName` to be converted.
    ///
    /// Returns an instance of `ApolloURDFLinkName`.
    pub fn from_link_name(link_name: &LinkName) -> Self {
        Self {
            link: link_name.link.clone(),
        }
    }
}

/// Struct representing an axis in URDF.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFAxis {
    /// The direction of the axis in the format [x, y, z].
    pub xyz: [f64; 3],
}

impl ApolloURDFAxis {
    /// Converts a URDF `Axis` to an `ApolloURDFAxis`.
    ///
    /// - `axis`: Reference to the URDF `Axis` to be converted.
    ///
    /// Returns an instance of `ApolloURDFAxis`.
    pub fn from_axis(axis: &Axis) -> Self {
        Self {
            xyz: axis.xyz.0,
        }
    }

    /// Creates a new `ApolloURDFAxis` with specified direction.
    ///
    /// - `xyz`: Direction of the axis [x, y, z].
    pub fn new(xyz: [f64; 3]) -> Self {
        Self { xyz }
    }
}

/// Struct representing the joint limit properties in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFJointLimit {
    /// The lower limit of the joint.
    pub lower: f64,

    /// The upper limit of the joint.
    pub upper: f64,

    /// The maximum effort of the joint.
    pub effort: f64,

    /// The maximum velocity of the joint.
    pub velocity: f64,
}

impl ApolloURDFJointLimit {
    /// Converts a URDF `JointLimit` to an `ApolloURDFJointLimit`.
    ///
    /// - `joint_limit`: Reference to the URDF `JointLimit` to be converted.
    ///
    /// Returns an instance of `ApolloURDFJointLimit`.
    pub fn from_joint_limit(joint_limit: &JointLimit) -> Self {
        Self {
            lower: joint_limit.lower,
            upper: joint_limit.upper,
            effort: joint_limit.effort,
            velocity: joint_limit.velocity,
        }
    }

    /// Creates a new `ApolloURDFJointLimit` with specified properties.
    ///
    /// - `lower`: Lower limit of the joint.
    /// - `upper`: Upper limit of the joint.
    /// - `effort`: Maximum effort.
    /// - `velocity`: Maximum velocity.
    pub fn new(lower: f64, upper: f64, effort: f64, velocity: f64) -> Self {
        Self { lower, upper, effort, velocity }
    }
}

impl Default for ApolloURDFJointLimit {
    fn default() -> Self {
        Self {
            lower: -3.14,
            upper: 3.14,
            effort: 0.0,
            velocity: 0.0,
        }
    }
}

/// Struct representing the dynamics properties of a joint in URDF.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFDynamics {
    /// Damping coefficient of the joint.
    pub damping: f64,

    /// Friction coefficient of the joint.
    pub friction: f64,
}

impl ApolloURDFDynamics {
    /// Converts a URDF `Dynamics` to an `ApolloURDFDynamics`.
    ///
    /// - `dynamics`: Reference to the URDF `Dynamics` to be converted.
    ///
    /// Returns an instance of `ApolloURDFDynamics`.
    pub fn from_dynamics(dynamics: &Dynamics) -> Self {
        Self {
            damping: dynamics.damping,
            friction: dynamics.friction,
        }
    }

    /// Creates a new `ApolloURDFDynamics` with specified properties.
    ///
    /// - `damping`: Damping coefficient.
    /// - `friction`: Friction coefficient.
    pub fn new(damping: f64, friction: f64) -> Self {
        Self { damping, friction }
    }
}

/// Struct representing the mimic properties of a joint in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFMimic {
    /// The name of the joint being mimicked.
    pub joint: String,

    /// The multiplier applied to the mimicked joint's motion.
    pub multiplier: Option<f64>,

    /// The offset added to the mimicked joint's motion.
    pub offset: Option<f64>,
}

impl ApolloURDFMimic {
    /// Converts a URDF `Mimic` to an `ApolloURDFMimic`.
    ///
    /// - `mimic`: Reference to the URDF `Mimic` to be converted.
    ///
    /// Returns an instance of `ApolloURDFMimic`.
    pub fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint: mimic.joint.clone(),
            multiplier: mimic.multiplier.clone(),
            offset: mimic.offset.clone(),
        }
    }

    /// Creates a new `ApolloURDFMimic` with specified properties.
    ///
    /// - `joint`: The name of the mimicked joint.
    /// - `multiplier`: Multiplier applied to the mimicked joint's motion.
    /// - `offset`: Offset added to the mimicked joint's motion.
    pub fn new(joint: String, multiplier: Option<f64>, offset: Option<f64>) -> Self {
        Self { joint, multiplier, offset }
    }
}

/// Struct representing the safety controller properties of a joint in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFSafetyController {
    /// The soft lower limit of the joint.
    pub soft_lower_limit: f64,

    /// The soft upper limit of the joint.
    pub soft_upper_limit: f64,

    /// The position gain of the joint's safety controller.
    pub k_position: f64,

    /// The velocity gain of the joint's safety controller.
    pub k_velocity: f64,
}

impl ApolloURDFSafetyController {
    /// Converts a URDF `SafetyController` to an `ApolloURDFSafetyController`.
    ///
    /// - `safety_controller`: Reference to the URDF `SafetyController` to be converted.
    ///
    /// Returns an instance of `ApolloURDFSafetyController`.
    pub fn from_safety_controller(safety_controller: &SafetyController) -> Self {
        Self {
            soft_lower_limit: safety_controller.soft_lower_limit,
            soft_upper_limit: safety_controller.soft_upper_limit,
            k_position: safety_controller.k_position,
            k_velocity: safety_controller.k_velocity,
        }
    }

    /// Creates a new `ApolloURDFSafetyController` with specified properties.
    ///
    /// - `soft_lower_limit`: Soft lower limit.
    /// - `soft_upper_limit`: Soft upper limit.
    /// - `k_position`: Position gain.
    /// - `k_velocity`: Velocity gain.
    pub fn new(soft_lower_limit: f64, soft_upper_limit: f64, k_position: f64, k_velocity: f64) -> Self {
        Self { soft_lower_limit, soft_upper_limit, k_position, k_velocity }
    }
}

/// Struct representing the color property of a material in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFColor {
    /// The RGBA values of the color.
    pub rgba: [f64; 4],
}

impl ApolloURDFColor {
    /// Converts a URDF `Color` to an `ApolloURDFColor`.
    ///
    /// - `color`: Reference to the URDF `Color` to be converted.
    ///
    /// Returns an instance of `ApolloURDFColor`.
    pub fn from_color(color: &Color) -> Self {
        Self {
            rgba: color.rgba.0,
        }
    }
}

/// Struct representing the texture property of a material in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFTexture {
    /// The filename of the texture.
    pub filename: String,
}

impl ApolloURDFTexture {
    /// Converts a URDF `Texture` to an `ApolloURDFTexture`.
    ///
    /// - `texture`: Reference to the URDF `Texture` to be converted.
    ///
    /// Returns an instance of `ApolloURDFTexture`.
    pub fn from_texture(texture: &Texture) -> Self {
        Self {
            filename: texture.filename.clone(),
        }
    }
}

/// Struct representing the mass property of a link in URDF.
#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFMass {
    /// The mass value.
    pub value: f64,
}

impl ApolloURDFMass {
    /// Converts a URDF `Mass` to an `ApolloURDFMass`.
    ///
    /// - `mass`: Reference to the URDF `Mass` to be converted.
    ///
    /// Returns an instance of `ApolloURDFMass`.
    pub fn from_mass(mass: &Mass) -> Self {
        Self {
            value: mass.value,
        }
    }
}

/// Struct representing the inertia matrix of a link in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertia {
    /// The moment of inertia around the X axis.
    pub ixx: f64,

    /// The product of inertia for the X-Y axes.
    pub ixy: f64,

    /// The product of inertia for the X-Z axes.
    pub ixz: f64,

    /// The moment of inertia around the Y axis.
    pub iyy: f64,

    /// The product of inertia for the Y-Z axes.
    pub iyz: f64,

    /// The moment of inertia around the Z axis.
    pub izz: f64,
}

impl ApolloURDFInertia {
    /// Converts a URDF `Inertia` to an `ApolloURDFInertia`.
    ///
    /// - `inertia`: Reference to the URDF `Inertia` to be converted.
    ///
    /// Returns an instance of `ApolloURDFInertia`.
    pub fn from_inertia(inertia: &Inertia) -> Self {
        Self {
            ixx: inertia.ixx,
            ixy: inertia.ixy,
            ixz: inertia.ixz,
            iyy: inertia.iyy,
            iyz: inertia.iyz,
            izz: inertia.izz,
        }
    }
}

impl Default for ApolloURDFInertia {
    fn default() -> Self {
        Self {
            ixx: 1.0,
            ixy: 0.0,
            ixz: 0.0,
            iyy: 1.0,
            iyz: 0.0,
            izz: 1.0,
        }
    }
}

/// Enum representing different types of geometry in URDF.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum ApolloURDFGeometry {
    Box {
        size: [f64; 3],
    },
    Cylinder {
        radius: f64,
        length: f64,
    },
    Capsule {
        radius: f64,
        length: f64,
    },
    Sphere {
        radius: f64,
    },
    Mesh {
        filename: String,
        scale: Option<[f64; 3]>,
    },
}

impl ApolloURDFGeometry {
    /// Converts a URDF `Geometry` to an `ApolloURDFGeometry`.
    ///
    /// - `geometry`: Reference to the URDF `Geometry` to be converted.
    ///
    /// Returns an instance of `ApolloURDFGeometry`.
    pub fn from_geometry(geometry: &Geometry) -> Self {
        match geometry {
            Geometry::Box { size } => Self::Box { size: size.0 },
            Geometry::Cylinder { radius, length } => Self::Cylinder { radius: *radius, length: *length },
            Geometry::Capsule { radius, length } => Self::Capsule { radius: *radius, length: *length },
            Geometry::Sphere { radius } => Self::Sphere { radius: *radius },
            Geometry::Mesh { filename, scale } => Self::Mesh {
                filename: filename.clone(),
                scale: scale.map(|s| s.0),
            },
        }
    }
}


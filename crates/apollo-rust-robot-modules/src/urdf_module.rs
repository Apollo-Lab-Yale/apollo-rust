use serde::{Deserialize, Serialize};
use urdf_rs::*;

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFModule {
    pub name: String,
    pub links: Vec<ApolloURDFLink>,
    pub joints: Vec<ApolloURDFJoint>,
    pub materials: Vec<ApolloURDFMaterial>
}
impl ApolloURDFModule {
    pub fn link_has_mesh(&self, link_idx: usize) -> bool {
        return !self.links[link_idx].visual.is_empty()
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFLink {
    pub name: String,
    pub inertial: ApolloURDFInertial,
    pub visual: Vec<ApolloURDFVisual>,
    pub collision: Vec<ApolloURDFCollision>
}
impl ApolloURDFLink {
    pub fn from_link(link: &Link) -> Self {
        Self {
            name: link.name.clone(),
            inertial: ApolloURDFInertial::from_inertial(&link.inertial),
            visual: link.visual.iter().map(|x| ApolloURDFVisual::from_visual(x) ).collect(),
            collision: link.collision.iter().map(|x| ApolloURDFCollision::from_collision(x)).collect(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFJoint {
    pub name: String,
    pub joint_type: ApolloURDFJointType,
    pub origin: ApolloURDFPose,
    pub parent: ApolloURDFLinkName,
    pub child: ApolloURDFLinkName,
    pub axis: ApolloURDFAxis,
    pub limit: ApolloURDFJointLimit,
    pub dynamics: Option<ApolloURDFDynamics>,
    pub mimic: Option<ApolloURDFMimic>,
    pub safety_controller: Option<ApolloURDFSafetyController>
}
impl ApolloURDFJoint {
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
                None => { None }
                Some(s) => { Some(ApolloURDFDynamics::from_dynamics(&s)) }
            },
            mimic: match &joint.mimic {
                None => { None }
                Some(s) => { Some(ApolloURDFMimic::from_mimic(&s)) }
            },
            safety_controller: match &joint.safety_controller {
                None => { None }
                Some(s) => { Some(ApolloURDFSafetyController::from_safety_controller(&s)) }
            },
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFMaterial {
    pub name: String,
    pub color: Option<ApolloURDFColor>,
    pub texture: Option<ApolloURDFTexture>
}
impl ApolloURDFMaterial {
    pub fn from_material(material: &Material) -> Self {
        Self {
            name: material.name.clone(),
            color: match &material.color {
                None => { None }
                Some(c) => { Some(ApolloURDFColor::from_color(c)) }
            },
            texture: match &material.texture {
                None => { None }
                Some(t) => { Some(ApolloURDFTexture::from_texture(t)) }
            },
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFInertial {
    pub origin: ApolloURDFPose,
    pub mass: ApolloURDFMass,
    pub inertia: ApolloURDFInertia,
}
impl ApolloURDFInertial {
    pub fn from_inertial(inertial: &Inertial) -> Self {
        Self {
            origin: ApolloURDFPose::from_pose(&inertial.origin),
            mass: ApolloURDFMass::from_mass(&inertial.mass),
            inertia: ApolloURDFInertia::from_inertia(&inertial.inertia),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFVisual {
    pub name: Option<String>,
    pub origin: ApolloURDFPose,
    pub geometry: ApolloURDFGeometry,
    pub material: Option<ApolloURDFMaterial>
}
impl ApolloURDFVisual {
    pub fn from_visual(visual: &Visual) -> Self {
        Self {
            name: visual.name.clone(),
            origin: ApolloURDFPose::from_pose(&visual.origin),
            geometry: ApolloURDFGeometry::from_geometry(&visual.geometry),
            material: match &visual.material {
                None => { None }
                Some(m) => { Some(ApolloURDFMaterial::from_material(m)) }
            },
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFCollision {
    pub name: Option<String>,
    pub origin: ApolloURDFPose,
    pub geometry: ApolloURDFGeometry
}
impl ApolloURDFCollision {
    pub fn from_collision(collision: &Collision) -> Self {
        Self {
            name: collision.name.clone(),
            origin: ApolloURDFPose::from_pose(&collision.origin),
            geometry: ApolloURDFGeometry::from_geometry(&collision.geometry),
        }
    }
}

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
    pub fn from_joint_type(joint_type: &JointType) -> Self {
        match joint_type {
            JointType::Revolute => { Self::Revolute }
            JointType::Continuous => { Self::Continuous  }
            JointType::Prismatic => { Self::Prismatic  }
            JointType::Fixed => { Self::Fixed  }
            JointType::Floating => { Self::Floating  }
            JointType::Planar => { Self::Planar  }
            JointType::Spherical => { Self::Spherical  }
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFPose {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3]
}
impl ApolloURDFPose {
    pub fn from_pose(pose: &Pose) -> Self {
        Self {
            xyz: pose.xyz.0,
            rpy: pose.rpy.0,
        }
    }
    pub fn new(xyz: [f64; 3], rpy: [f64; 3]) -> Self {
        Self { xyz, rpy }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFLinkName {
    pub link: String
}
impl ApolloURDFLinkName {
    pub fn from_link_name(link_name: &LinkName) -> Self {
        Self {
            link: link_name.link.clone(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFAxis {
    pub xyz: [f64; 3]
}
impl ApolloURDFAxis {
    pub fn from_axis(axis: &Axis) -> Self {
        Self {
            xyz: axis.xyz.0,
        }
    }
    pub fn new(xyz: [f64; 3]) -> Self {
        Self { xyz }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFJointLimit {
    pub lower: f64,
    pub upper: f64,
    pub effort: f64,
    pub velocity: f64
}
impl ApolloURDFJointLimit {
    pub fn from_joint_limit(joint_limit: &JointLimit) -> Self {
        Self {
            lower: joint_limit.lower,
            upper: joint_limit.upper,
            effort: joint_limit.effort,
            velocity: joint_limit.velocity,
        }
    }
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

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFDynamics {
    pub damping: f64,
    pub friction: f64
}
impl ApolloURDFDynamics {
    pub fn from_dynamics(dynamics: &Dynamics) -> Self {
        Self {
            damping: dynamics.damping,
            friction: dynamics.friction,
        }
    }
    pub fn new(damping: f64, friction: f64) -> Self {
        Self { damping, friction }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFMimic {
    pub joint: String,
    pub multiplier: Option<f64>,
    pub offset: Option<f64>
}
impl ApolloURDFMimic {
    pub fn from_mimic(mimic: &Mimic) -> Self {
        Self {
            joint: mimic.joint.clone(),
            multiplier: mimic.multiplier.clone(),
            offset: mimic.offset.clone(),
        }
    }
    pub fn new(joint: String, multiplier: Option<f64>, offset: Option<f64>) -> Self {
        Self { joint, multiplier, offset }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFSafetyController {
    pub soft_lower_limit: f64,
    pub soft_upper_limit: f64,
    pub k_position: f64,
    pub k_velocity: f64,
}
impl ApolloURDFSafetyController {
    pub fn from_safety_controller(safety_controller: &SafetyController) -> Self {
        Self {
            soft_lower_limit: safety_controller.soft_lower_limit,
            soft_upper_limit: safety_controller.soft_upper_limit,
            k_position: safety_controller.k_position,
            k_velocity: safety_controller.k_velocity
        }
    }
    pub fn new(soft_lower_limit: f64, soft_upper_limit: f64, k_position: f64, k_velocity: f64) -> Self {
        Self { soft_lower_limit, soft_upper_limit, k_position, k_velocity }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFColor {
    pub rgba: [f64; 4],
}
impl ApolloURDFColor {
    pub fn from_color(color: &Color) -> Self {
        Self {
            rgba: color.rgba.0,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFTexture {
    pub filename: String,
}
impl ApolloURDFTexture {
    pub fn from_texture(texture: &Texture) -> Self {
        Self {
            filename: texture.filename.clone(),
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, Default)]
pub struct ApolloURDFMass {
    pub value: f64,
}
impl ApolloURDFMass {
    pub fn from_mass(mass: &Mass) -> Self {
        Self {
            value: mass.value,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloURDFInertia {
    pub ixx: f64,
    pub ixy: f64,
    pub ixz: f64,
    pub iyy: f64,
    pub iyz: f64,
    pub izz: f64,
}
impl ApolloURDFInertia {
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
    pub fn from_geometry(geometry: &Geometry) -> Self {
        match geometry {
            Geometry::Box { size } => {
                Self::Box { size: size.0 }
            }
            Geometry::Cylinder { radius, length } => {
                Self::Cylinder { radius: *radius, length: *length }
            }
            Geometry::Capsule { radius, length } => {
                Self::Capsule {
                    radius: *radius,
                    length: *length,
                }
            }
            Geometry::Sphere { radius } => {
                Self::Sphere {
                    radius: *radius,
                }
            }
            Geometry::Mesh { filename, scale } => {
                Self::Mesh {
                    filename: filename.clone(),
                    scale: match scale {
                        None => { None }
                        Some(s) => { Some(s.0) }
                    },
                }
            }
        }
    }
}
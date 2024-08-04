use std::path::PathBuf;
use nalgebra::Isometry3;
use serde::{Deserialize, Serialize};
use apollo_rust_environment_modules::environment_link_simulation_modes_module::EnvironmentLinkSimulationMode;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFAxis, ApolloURDFJointLimit, ApolloURDFJointType};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentCreator {
    pub environment_name: String,
    pub actions: Vec<EnvironmentCreatorAction>
}
impl ApolloEnvironmentCreator {

}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentCreatorAction {
    AddSingleObjectFromStlFile { fp: PathBuf, object_name: String, parent_object: Option<String>, transform: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromObjFile { fp: PathBuf, object_name: String, parent_object: Option<String>, transform: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromGlbFile { fp: PathBuf, object_name: String, parent_object: Option<String>, transform: ISE3q, scale: [f64; 3] },
    AddSceneFromGlbFile { fp: PathBuf, parent_object: Option<String>, transform: ISE3q, scale: [f64; 3] },
    SetJointType { parent_object: Option<String>, child_object: String, joint_type: ApolloURDFJointType },
    SetJointAxis { parent_object: Option<String>, child_object: String, axis: ApolloURDFAxis },
    SetJointLimit { parent_object: Option<String>, child_object: String, joint_limit: ApolloURDFJointLimit },
    SetObjectSimulationMode { object_name: String, mode: EnvironmentLinkSimulationMode },
    SetObjectMass { object_name: String, mass: f64 },
    SetObjectInertialOrigin { object_name: String, pose: ISE3q  },
    SetObjectInertia { object_name: String, ixx: f64, ixy: f64, ixz: f64, iyy: f64, iyz: f64, izz: f64 }
}
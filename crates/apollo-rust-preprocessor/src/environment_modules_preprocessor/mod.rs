pub mod modules;

use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use apollo_rust_environment_modules::environment_link_simulation_modes_module::EnvironmentLinkSimulationMode;
use apollo_rust_environment_modules::{ResourcesEnvironmentsDirectory, ResourcesSingleEnvironmentDirectory};
use apollo_rust_robot_modules::urdf_module::{ApolloURDFAxis, ApolloURDFJointLimit, ApolloURDFJointType};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::{ResourcesRootDirectoryPreprocessorTrait, ResourcesRootDirectoryTrait, ResourcesSubDirectoryPreprocessorTrait, ResourcesSubDirectoryTrait};

impl ResourcesRootDirectoryTrait for ResourcesEnvironmentsDirectory {
    type SubDirectoryType = ResourcesSingleEnvironmentDirectory;

    fn new(directory: PathBuf) -> Self {
        Self {
            directory,
        }
    }

    fn new_default() -> Self {
        todo!()
    }

    fn directory(&self) -> &PathBuf {
        &self.directory
    }
}

impl ResourcesRootDirectoryPreprocessorTrait for ResourcesEnvironmentsDirectory {
    fn preprocess_all(&self, force_build_on_all: bool) {
        self.get_all_subdirectories().iter().for_each(|x| x.preprocess(force_build_on_all));
    }
}

impl ResourcesSubDirectoryTrait for ResourcesSingleEnvironmentDirectory {
    fn new_raw(name: String, root_directory: PathBuf, directory: PathBuf) -> Self {
        Self {
            environment_name: name,
            environments_directory: root_directory,
            directory,
        }
    }

    fn name(&self) -> String {
        self.environment_name.clone()
    }

    fn root_directory(&self) -> &PathBuf {
        &self.environments_directory
    }

    fn directory(&self) -> &PathBuf {
        &self.directory
    }
}

impl ResourcesSubDirectoryPreprocessorTrait for ResourcesSingleEnvironmentDirectory {
    fn preprocess(&self, _force_build_on_all: bool) {
        todo!()
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentCreator {
    actions: Vec<EnvironmentCreatorAction>
}
impl ApolloEnvironmentCreator {
    pub fn new() -> Self {
        Self {
            actions: vec![],
        }
    }
    pub fn add_action(&mut self, action: EnvironmentCreatorAction) {
        for a in &self.actions {
            if a.is_matching(&action) {
                println!("already have an action with signature {:?}.  Cannot add duplicate.  Returning.", a.to_signature());
                return;
            }
        }

        self.actions.push(action);
    }
    pub fn add_or_replace(&mut self, action: EnvironmentCreatorAction) {
        let idx = self.actions.iter().position(|x| x.is_matching(&action));
        match idx {
            None => { self.actions.push(action); }
            Some(idx) => { self.actions[idx] = action; }
        }
    }
    pub fn remove_action(&mut self, signature: EnvironmentCreatorActionSignature) {
        let idx = self.actions.iter().position(|x| x.is_matching_signature(&signature));
        match idx {
            None => { }
            Some(idx) => { self.actions.remove(idx); }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentCreatorAction {
    AddAlreadyExistingEnvironment { name: String, base_offset: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromStlFile { fp: PathBuf, object_name: String, parent_object: Option<String>, base_offset: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromObjFile { fp: PathBuf, object_name: String, parent_object: Option<String>, base_offset: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromGlbFile { fp: PathBuf, object_name: String, parent_object: Option<String>, base_offset: ISE3q, scale: [f64; 3] },
    AddSceneFromGlbFile { fp: PathBuf, parent_object: Option<String>, transform: ISE3q, scale: [f64; 3] },
    SetJointType { parent_object: Option<String>, child_object: String, joint_type: ApolloURDFJointType },
    SetJointAxis { parent_object: Option<String>, child_object: String, axis: ApolloURDFAxis },
    SetJointLimit { parent_object: Option<String>, child_object: String, joint_limit: ApolloURDFJointLimit },
    SetObjectSimulationMode { object_name: String, mode: EnvironmentLinkSimulationMode },
    SetObjectMass { object_name: String, mass: f64 },
    SetObjectInertialOrigin { object_name: String, pose: ISE3q  },
    SetObjectInertia { object_name: String, ixx: f64, ixy: f64, ixz: f64, iyy: f64, iyz: f64, izz: f64 }
}
impl EnvironmentCreatorAction {
    pub fn to_signature(&self) -> EnvironmentCreatorActionSignature {
        match self {
            EnvironmentCreatorAction::AddAlreadyExistingEnvironment { name, .. } => { EnvironmentCreatorActionSignature::AddAlreadyExistingEnvironment { name: name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromStlFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromStlFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromObjFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromObjFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromGlbFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromGlbFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSceneFromGlbFile { fp, .. } => { EnvironmentCreatorActionSignature::AddSceneFromGlbFile { fp: fp.clone() } }
            EnvironmentCreatorAction::SetJointType { parent_object, child_object, .. } => {  EnvironmentCreatorActionSignature::SetJointType { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            EnvironmentCreatorAction::SetJointAxis { parent_object, child_object, .. } => { EnvironmentCreatorActionSignature::SetJointAxis { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            EnvironmentCreatorAction::SetJointLimit { parent_object, child_object, .. } => { EnvironmentCreatorActionSignature::SetJointLimit { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            EnvironmentCreatorAction::SetObjectSimulationMode { object_name, .. } => { EnvironmentCreatorActionSignature::SetObjectSimulationMode { object_name: object_name.clone() } }
            EnvironmentCreatorAction::SetObjectMass { object_name, .. } => { EnvironmentCreatorActionSignature::SetObjectMass { object_name: object_name.clone() } }
            EnvironmentCreatorAction::SetObjectInertialOrigin { object_name, .. } => { EnvironmentCreatorActionSignature::SetObjectInertialOrigin { object_name: object_name.clone() } }
            EnvironmentCreatorAction::SetObjectInertia { object_name, .. } => { EnvironmentCreatorActionSignature::SetObjectInertia { object_name: object_name.clone() } }
        }
    }

    pub fn is_matching(&self, other: &EnvironmentCreatorAction) -> bool {
        return self.to_signature() == other.to_signature()
    }

    pub fn is_matching_signature(&self, signature: &EnvironmentCreatorActionSignature) -> bool {
        &self.to_signature() == signature
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum EnvironmentCreatorActionSignature {
    AddAlreadyExistingEnvironment { name: String },
    AddSingleObjectFromStlFile { object_name: String },
    AddSingleObjectFromObjFile { object_name: String },
    AddSingleObjectFromGlbFile { object_name: String },
    AddSceneFromGlbFile { fp: PathBuf },
    SetJointType { parent_object: Option<String>, child_object: String },
    SetJointAxis { parent_object: Option<String>, child_object: String },
    SetJointLimit { parent_object: Option<String>, child_object: String },
    SetObjectSimulationMode { object_name: String },
    SetObjectMass { object_name: String },
    SetObjectInertialOrigin { object_name: String },
    SetObjectInertia { object_name: String }
}
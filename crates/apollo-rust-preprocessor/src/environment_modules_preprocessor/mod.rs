pub mod modules;

use std::path::PathBuf;
use serde::{Deserialize, Serialize};
use apollo_rust_environment_modules::{ResourcesEnvironmentsDirectory, ResourcesSingleEnvironmentDirectory};
use apollo_rust_environment_modules::environment_description_module::{ApolloEnvironmentDescriptionModule, EnvironmentLinkSimulationMode};
use apollo_rust_environment_modules::mesh_modules::environment_convex_decomposition_meshes_module::ApolloEnvironmentConvexDecompositionMeshesModule;
use apollo_rust_environment_modules::mesh_modules::environment_convex_hull_meshes_module::ApolloEnvironmentConvexHullMeshesModule;
use apollo_rust_environment_modules::mesh_modules::environment_original_meshes_module::ApolloEnvironmentOriginalMeshesModule;
use apollo_rust_environment_modules::mesh_modules::environment_plain_meshes_module::ApolloEnvironmentPlainMeshesModule;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFAxis, ApolloURDFJointLimit, ApolloURDFJointType};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::{PreprocessorModule, ResourcesRootDirectoryPreprocessorTrait, ResourcesRootDirectoryTrait, ResourcesSubDirectoryPreprocessorTrait, ResourcesSubDirectoryTrait};

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
    fn preprocess(&self, force_build_on_all: bool) {
        ApolloEnvironmentDescriptionModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloEnvironmentOriginalMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloEnvironmentPlainMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloEnvironmentConvexHullMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloEnvironmentConvexDecompositionMeshesModule::load_or_build(self, force_build_on_all).expect("error");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentCreator {
    name: String,
    actions: Vec<EnvironmentCreatorAction>
}
impl ApolloEnvironmentCreator {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            actions: vec![],
        }
    }
    pub fn to_new_name(&self, new_name: &str) -> Self {
        let mut out = self.clone();
        out.name = new_name.to_string();
        return out;
    }
    pub fn add_action(self, action: EnvironmentCreatorAction) -> Self {
        let mut out = self.clone();

        for a in &out.actions {
            if a.is_matching(&action) {
                println!("already have an action with signature {:?}.  Cannot add duplicate.  Returning.", a.to_signature());
                return out;
            }
        }

        out.actions.push(action);

        out
    }
    pub fn add_or_replace(self, action: EnvironmentCreatorAction) -> Self {
        let mut out = self.clone();

        let idx = out.actions.iter().position(|x| x.is_matching(&action));
        match idx {
            None => { out.actions.push(action); }
            Some(idx) => { out.actions[idx] = action; }
        }

        out
    }
    pub fn remove_action(self, signature: EnvironmentCreatorActionSignature) -> Self {
        let mut out = self.clone();

        let idx = out.actions.iter().position(|x| x.is_matching_signature(&signature));
        match idx {
            None => { return out; }
            Some(idx) => { out.actions.remove(idx); }
        }

        return out;
    }
    pub fn create_and_preprocess(&self, r: &ResourcesEnvironmentsDirectory, force_build_on_all: bool) {
        let fp = r.directory().clone().append(&self.name);
        assert!(!fp.exists(), "environment with name {:?} already exists!", self.name);

        fp.create_directory();
        let s = ResourcesSingleEnvironmentDirectory {
            environment_name: self.name.clone(),
            environments_directory: r.directory.clone(),
            directory: fp.clone(),
        };

        let json_fp = fp.clone().append("creator_module/module.json");
        json_fp.save_object_to_json_file(self);
        let ron_fp =  fp.clone().append("creator_module/module.ron");
        ron_fp.save_object_to_ron_file(self);
        let yaml_fp =  fp.clone().append("creator_module/module.yaml");
        yaml_fp.save_object_to_yaml_file(self);

        s.preprocess(force_build_on_all);
    }
    pub fn load(r: &ResourcesEnvironmentsDirectory, name: &str) -> Self {
        let s = r.get_subdirectory(name);
        let fp = s.directory.clone().append("creator_module/module.json");
        return fp.load_object_from_json_file()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentCreatorAction {
    AddAlreadyExistingEnvironment { name: String, base_offset: ISE3q, scale: [f64; 3] },
    AddSingleObjectFromStlFile {
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        object_name: String,
        parent_object: Option<String>,
        base_offset: ISE3q,
        scale: [f64; 3]
    },
    AddSingleObjectFromObjFile {
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        object_name: String,
        parent_object: Option<String>,
        base_offset: ISE3q,
        scale: [f64; 3]
    },
    AddSingleObjectFromGlbFile {
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        object_name: String,
        parent_object: Option<String>,
        base_offset: ISE3q,
        scale: [f64; 3]
    },
    AddSceneFromGlbFile {
        scene_name: String,
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        parent_object: Option<String>,
        transform: ISE3q,
        scale: [f64; 3]
    },
    SetJointType { parent_object: Option<String>, child_object: String, joint_type: ApolloURDFJointType },
    SetJointAxis { parent_object: Option<String>, child_object: String, axis: ApolloURDFAxis },
    SetJointLimit { parent_object: Option<String>, child_object: String, joint_limit: ApolloURDFJointLimit },
    SetObjectSimulationMode { object_name: String, mode: EnvironmentLinkSimulationMode },
    SetObjectMass { object_name: String, mass: f64 },
    SetObjectInertialOrigin { object_name: String, pose: ISE3q },
    SetObjectInertia { object_name: String, ixx: f64, ixy: f64, ixz: f64, iyy: f64, iyz: f64, izz: f64 }
}
impl EnvironmentCreatorAction {
    pub fn to_signature(&self) -> EnvironmentCreatorActionSignature {
        match self {
            EnvironmentCreatorAction::AddAlreadyExistingEnvironment { name, .. } => { EnvironmentCreatorActionSignature::AddAlreadyExistingEnvironment { name: name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromStlFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromStlFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromObjFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromObjFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSingleObjectFromGlbFile { object_name, .. } => { EnvironmentCreatorActionSignature::AddSingleObjectFromGlbFile { object_name: object_name.clone() } }
            EnvironmentCreatorAction::AddSceneFromGlbFile { scene_name, .. } => { EnvironmentCreatorActionSignature::AddSceneFromGlbFile { scene_name: scene_name.to_string() } }
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
    AddSceneFromGlbFile { scene_name: String },
    SetJointType { parent_object: Option<String>, child_object: String },
    SetJointAxis { parent_object: Option<String>, child_object: String },
    SetJointLimit { parent_object: Option<String>, child_object: String },
    SetObjectSimulationMode { object_name: String },
    SetObjectMass { object_name: String },
    SetObjectInertialOrigin { object_name: String },
    SetObjectInertia { object_name: String }
}
pub mod modules;
pub mod utils;

use serde::{Deserialize, Serialize};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_robot_modules::robot_modules::urdf_module::{ApolloURDFAxis, ApolloURDFDynamics, ApolloURDFJoint, ApolloURDFJointLimit, ApolloURDFJointType, ApolloURDFLink, ApolloURDFModule, ApolloURDFPose, ApolloURDFSafetyController};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use std::path::PathBuf;
use apollo_rust_robot_modules::robot_modules::link_simulation_mode_module::EnvironmentLinkSimulationMode;
use crate::{PreprocessorModule, ResourcesSubDirectoryTrait};

/*
pub trait ResourcesSingleRobotDirectoryTrait {
    fn preprocess(&self, force_build_on_all: bool);
}
impl ResourcesSingleRobotDirectoryTrait for ResourcesSingleRobotDirectory {
    fn preprocess(&self, force_build_on_all: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloOriginalMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloPlainMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloBoundsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesDistanceStatisticsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesSimpleSkipsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesApproximationsModule::load_or_build(self, force_build_on_all).expect("error");
    }
}

pub trait ResourcesRobotsDirectoryTrait {
    fn preprocess_all(&self, force_build_on_all: bool);
}
impl ResourcesRobotsDirectoryTrait for ResourcesRobotsDirectory {
    fn preprocess_all(&self, force_build_on_all: bool) {
        for x in self.get_all_subdirectories() {
            x.preprocess(force_build_on_all);
        }
    }
}
*/

/*
impl ResourcesRootDirectoryTrait for ResourcesRobotsDirectory {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn new(directory: PathBuf) -> Self {
        Self {
            directory,
        }
    }

    fn new_default() -> Self {
        let out = PathBuf::new_from_default_apollo_robots_dir();
        assert!(out.exists());
        Self {
            directory: out,
        }
    }

    fn directory(&self) -> &PathBuf {
        &self.directory
    }
}

impl ResourcesSubDirectoryTrait for ResourcesSingleRobotDirectory {
    fn new_raw(name: String, root_directory: PathBuf, directory: PathBuf) -> Self {
        Self {
            robot_name: name.clone(),
            robots_directory: root_directory.clone(),
            directory,
        }
    }

    fn name(&self) -> String {
        self.robot_name.clone()
    }

    fn root_directory(&self) -> &PathBuf {
        &self.robots_directory
    }

    fn directory(&self) -> &PathBuf {
        &self.directory
    }
}

impl ResourcesRootDirectoryPreprocessorTrait for ResourcesRobotsDirectory {
    fn preprocess_all(&self, force_build_on_all: bool) {
        for x in self.get_all_subdirectories() {
            x.preprocess(force_build_on_all);
        }
    }
}

impl ResourcesSubDirectoryPreprocessorTrait for ResourcesSingleRobotDirectory {
    fn preprocess(&self, force_build_on_all: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloOriginalMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloPlainMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloBoundsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesDistanceStatisticsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesSimpleSkipsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloLinkShapesApproximationsModule::load_or_build(self, force_build_on_all).expect("error");
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

/*
pub trait RobotPreprocessorModule: Serialize + DeserializeOwned {
    fn relative_file_path_str_from_robot_sub_dir_to_module_dir() -> String;

    fn relative_file_path_from_root_dir_to_module_dir(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        PathBuf::new().append(&s.name()).append(&Self::relative_file_path_str_from_robot_sub_dir_to_module_dir())
    }

    fn full_path_to_module_dir(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        s.directory.clone().append(&Self::relative_file_path_str_from_robot_sub_dir_to_module_dir())
    }

    fn full_path_to_module_version(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("VERSION")
    }

    fn full_path_module_json(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.json")
    }

    fn full_path_module_ron(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.ron")
    }

    fn full_path_module_yaml(s: &ResourcesSingleRobotDirectory) -> PathBuf {
        Self::full_path_to_module_dir(s).append("module.yaml")
    }

    /// should be in the format "0.0.1"
    fn current_version() -> String;

    fn build_raw(s: &ResourcesSingleRobotDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String>;

    fn build(s: &ResourcesSingleRobotDirectory) -> Result<Self, String> {
        let mut pb = ProgressBarWrapper::new(&s.name(), &Self::relative_file_path_str_from_robot_sub_dir_to_module_dir());
        let o = Self::build_raw(s, &mut pb);
        return match o {
            Ok(o) => {
                o.save(&s);
                Ok(o)
            }
            Err(e) => {
                Err(e)
            }
        }
    }

    fn load_from_json(s: &ResourcesSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_json(s).load_object_from_json_file_result()
    }

    fn load_from_ron(s: &ResourcesSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_ron(s).load_object_from_ron_file_result()
    }

    fn load_from_yaml(s: &ResourcesSingleRobotDirectory) -> Result<Self, String> {
        let saved_version = Self::full_path_to_module_version(s).read_file_contents_to_string();
        if saved_version != Self::current_version() { return Err(format!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version())) }
        Self::full_path_module_yaml(s).load_object_from_yaml_file_result()
    }

    fn save(&self, s: &ResourcesSingleRobotDirectory) {
        Self::full_path_to_module_version(s).write_string_to_file(&Self::current_version());
        Self::full_path_module_json(s).save_object_to_json_file(self);
        Self::full_path_module_ron(s).save_object_to_ron_file(self);
        Self::full_path_module_yaml(s).save_object_to_yaml_file(self);
    }

    fn load_or_build(s: &ResourcesSingleRobotDirectory, force_build: bool) -> Result<Self, String> {
        if !force_build {
            let fp = Self::full_path_to_module_version(s);
            match fp.exists() {
                true => {
                    let saved_version = fp.read_file_contents_to_string();
                    if saved_version == Self::current_version() {
                        let loaded = Self::load_from_json(s);
                        match loaded {
                            Ok(loaded) => { return Ok(loaded); }
                            Err(e) => {
                                println!("Unable to load module in {:?} because of this reason: {:?}.  Will rebuild.", Self::full_path_module_json(s), e);
                            }
                        }
                    } else {
                        println!("Version did not match when loading module {:?}.  saved version: {:?} vs. current version: {:?}.  I will rebuild this module.", Self::relative_file_path_str_from_robot_sub_dir_to_module_dir(), saved_version, Self::current_version());
                    }
                }
                false => { }
            }
        }

        return Self::build(s);
    }
}
*/

////////////////////////////////////////////////////////////////////////////////////////////////////

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CombinedRobot {
    name: String,
    attached_robots: Vec<AttachedRobot>
}
impl CombinedRobot {
    pub fn new(name: &str) -> Self {
        Self { name: name.to_string(), attached_robots: vec![] }
    }
    pub fn attach_robot(self,
                        robot_name: &str,
                        attachment_point: AttachmentPoint,
                        joint_type: ApolloURDFJointType,
                        origin: ApolloURDFPose,
                        axis: ApolloURDFAxis,
                        limit: ApolloURDFJointLimit,
                        dynamics: Option<ApolloURDFDynamics>,
                        safety_controller: Option<ApolloURDFSafetyController>) -> Self {
        let mut out = self.clone();

        match &attachment_point {
            AttachmentPoint::Link { robot_idx, .. } => { assert!(*robot_idx < out.attached_robots.len()) }
            _ => { }
        }

        out.attached_robots.push(
            AttachedRobot {
                robot_name: robot_name.to_string(),
                attachment_point,
                joint_type,
                origin,
                axis,
                limit,
                dynamics,
                safety_controller
            }
        );

        out
    }
    pub fn attach_robot_fixed(self, robot_name: &str, attachment_point: AttachmentPoint, origin: ApolloURDFPose) -> Self {
        Self::attach_robot(self, robot_name, attachment_point, ApolloURDFJointType::Fixed, origin, ApolloURDFAxis::default(), ApolloURDFJointLimit::default(), None, None)
    }
    pub fn attached_robots(&self) -> &Vec<AttachedRobot> {
        &self.attached_robots
    }
    pub fn create_and_preprocess(&self, r: &ResourcesRootDirectory) {
        let fp = r.directory().clone().append(&self.name);
        assert!( !fp.exists(), "combined robot directory with name {:?} already exists!", self.name );

        fp.create_directory();
        let s = ResourcesSubDirectory {
            name: self.name.clone(),
            root_directory: r.directory.clone(),
            directory: fp.clone(),
        };

        let json_fp = fp.clone().append("combined_robot_module/module.json");
        json_fp.save_object_to_json_file(self);
        let ron_fp =  fp.clone().append("combined_robot_module/module.ron");
        ron_fp.save_object_to_ron_file(self);
        let yaml_fp =  fp.clone().append("combined_robot_module/module.yaml");
        yaml_fp.save_object_to_yaml_file(self);

        s.preprocess_robot(false);
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AttachedRobot {
    robot_name: String,
    attachment_point: AttachmentPoint,
    joint_type: ApolloURDFJointType,
    origin: ApolloURDFPose,
    axis: ApolloURDFAxis,
    limit: ApolloURDFJointLimit,
    dynamics: Option<ApolloURDFDynamics>,
    safety_controller: Option<ApolloURDFSafetyController>
}
impl AttachedRobot {
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }
    pub fn attachment_point(&self) -> &AttachmentPoint {
        &self.attachment_point
    }
    pub fn joint_type(&self) -> &ApolloURDFJointType {
        &self.joint_type
    }
    pub fn origin(&self) -> &ApolloURDFPose {
        &self.origin
    }
    pub fn axis(&self) -> &ApolloURDFAxis {
        &self.axis
    }
    pub fn limit(&self) -> &ApolloURDFJointLimit {
        &self.limit
    }
    pub fn dynamics(&self) -> &Option<ApolloURDFDynamics> {
        &self.dynamics
    }
    pub fn safety_controller(&self) -> &Option<ApolloURDFSafetyController> {
        &self.safety_controller
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum AttachmentPoint {
    World,
    Link { robot_idx: usize, link_name: String }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AdjustedRobot {
    name: String,
    base_robot_name: String,
    dead_end_link_names: Vec<String>,
    fixed_base_joints: Vec<(String, Vec<f64>)>,
    deleted_base_links: Vec<String>,
    deleted_base_joints: Vec<String>,
    adjusted_link_idx_to_base_link_idx_mapping: Vec<usize>,
    base_link_idx_to_adjusted_link_idx_mapping: Vec<Option<usize>>,
    adjusted_joint_idx_to_base_joint_idx_mapping: Vec<usize>,
    base_joint_idx_to_adjusted_joint_idx_mapping: Vec<Option<usize>>
}
impl AdjustedRobot {
    pub fn new(name: &str, base_robot_name: &str) -> Self {
        Self {
            name: name.to_string(),
            base_robot_name: base_robot_name.to_string(),
            dead_end_link_names: vec![],
            fixed_base_joints: vec![],
            deleted_base_links: vec![],
            deleted_base_joints: vec![],
            adjusted_link_idx_to_base_link_idx_mapping: vec![],
            base_link_idx_to_adjusted_link_idx_mapping: vec![],
            adjusted_joint_idx_to_base_joint_idx_mapping: vec![],
            base_joint_idx_to_adjusted_joint_idx_mapping: vec![],
        }
    }
    pub fn add_dead_end_link(self, link_name: &str) -> Self {
        let mut out = self.clone();

        out.dead_end_link_names.push(link_name.to_string());

        out
    }
    pub fn add_fixed_joint(self, joint_name: &str, fixed_dofs: &[f64]) -> Self {
        let mut out = self.clone();

        out.fixed_base_joints.push((joint_name.to_string(), fixed_dofs.to_vec()));

        out
    }
    pub fn create_and_preprocess(&self, r: &ResourcesRootDirectory) {
        let mut out = self.clone();

        let s = r.get_subdirectory(&self.base_robot_name);

        let urdf_module = ApolloURDFModule::load_or_build(&s, false).expect("error");
        let chain_module = ApolloChainModule::load_or_build(&s, false).expect("error");

        let mut deleted_base_links = vec![];
        let mut deleted_base_joints = vec![];

        self.dead_end_link_names.iter().for_each(|x| {
            let link_idx = urdf_module.links.iter().position(|y| &y.name == x).expect(&format!("link with name {} does not exist", x));
            let mut stack = vec![link_idx];

            while !stack.is_empty() {
                let curr_idx = stack.pop().unwrap();
                deleted_base_links.push(urdf_module.links[curr_idx].name.clone());
                let link_in_chain = &chain_module.links_in_chain[curr_idx];
                let parent_joint_idx = link_in_chain.parent_joint_idx();
                if let Some(parent_joint_idx) = parent_joint_idx {
                    deleted_base_joints.push(urdf_module.joints[parent_joint_idx].name.clone());
                }

                let children_links = link_in_chain.children_link_idxs();
                children_links.iter().for_each(|y| stack.push(*y));
            }
        });

        let adjusted_links: Vec<&ApolloURDFLink> = urdf_module.links.iter().filter(|y| !deleted_base_links.contains(&y.name)).collect();
        let adjusted_joints: Vec<&ApolloURDFJoint> = urdf_module.joints.iter().filter(|y| !deleted_base_joints.contains(&y.name)).collect();

        let mut adjusted_link_idx_to_base_link_idx_mapping = vec![];
        let mut base_link_idx_to_adjusted_link_idx_mapping = vec![];
        let mut adjusted_joint_idx_to_base_joint_idx_mapping = vec![];
        let mut base_joint_idx_to_adjusted_joint_idx_mapping = vec![];

        adjusted_links.iter().for_each(|x| {
            let base_link_idx = urdf_module.links.iter().position(|y| &y.name == &x.name).expect("error");
            adjusted_link_idx_to_base_link_idx_mapping.push(base_link_idx);
        });

        urdf_module.links.iter().for_each(|x| {
            let adjusted_link_idx = adjusted_links.iter().position(|y| &y.name == &x.name);
            match adjusted_link_idx {
                None => {base_link_idx_to_adjusted_link_idx_mapping.push(None)}
                Some(i) => {base_link_idx_to_adjusted_link_idx_mapping.push(Some(i));}
            }
        });

        adjusted_joints.iter().for_each(|x| {
            let base_joint_idx = urdf_module.joints.iter().position(|y| &y.name == &x.name).expect("error");
            adjusted_joint_idx_to_base_joint_idx_mapping.push(base_joint_idx);
        });

        urdf_module.joints.iter().for_each(|x| {
            let adjusted_joint_idx = adjusted_joints.iter().position(|y| &y.name == &x.name);
            match adjusted_joint_idx {
                None => {base_joint_idx_to_adjusted_joint_idx_mapping.push(None)}
                Some(i) => {base_joint_idx_to_adjusted_joint_idx_mapping.push(Some(i));}
            }
        });

        out.deleted_base_links = deleted_base_links;
        out.deleted_base_joints = deleted_base_joints;
        out.adjusted_link_idx_to_base_link_idx_mapping = adjusted_link_idx_to_base_link_idx_mapping;
        out.base_link_idx_to_adjusted_link_idx_mapping = base_link_idx_to_adjusted_link_idx_mapping;
        out.adjusted_joint_idx_to_base_joint_idx_mapping = adjusted_joint_idx_to_base_joint_idx_mapping;
        out.base_joint_idx_to_adjusted_joint_idx_mapping = base_joint_idx_to_adjusted_joint_idx_mapping;

        let fp = r.directory.clone().append(&self.name);
        assert!( !fp.exists(), "adjusted robot directory with name {:?} already exists!", self.name );

        fp.create_directory();

        let json_fp = fp.clone().append("adjusted_robot_module/module.json");
        json_fp.save_object_to_json_file(&out);
        let ron_fp =  fp.clone().append("adjusted_robot_module/module.ron");
        ron_fp.save_object_to_ron_file(&out);
        let yaml_fp =  fp.clone().append("adjusted_robot_module/module.yaml");
        yaml_fp.save_object_to_yaml_file(&out);

        let s = ResourcesSubDirectory {
            name: self.name.clone(),
            root_directory: r.directory.clone(),
            directory: fp.clone(),
        };
        s.preprocess_robot(false);
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloChainCreator {
    pub name: String,
    pub actions: Vec<ChainCreatorAction>
}
impl ApolloChainCreator {
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
    pub fn add_action(self, action: ChainCreatorAction) -> Self {
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
    pub fn add_or_replace(self, action: ChainCreatorAction) -> Self {
        let mut out = self.clone();

        let idx = out.actions.iter().position(|x| x.is_matching(&action));
        match idx {
            None => { out.actions.push(action); }
            Some(idx) => { out.actions[idx] = action; }
        }

        out
    }
    pub fn remove_action(self, signature: ChainCreatorActionSignature) -> Self {
        let mut out = self.clone();

        let idx = out.actions.iter().position(|x| x.is_matching_signature(&signature));
        match idx {
            None => { return out; }
            Some(idx) => { out.actions.remove(idx); }
        }

        return out;
    }
    pub fn create_and_preprocess(&self, r: &ResourcesRootDirectory, force_build_on_all: bool) {
        let fp = r.directory().clone().append(&self.name);
        assert!(!fp.exists(), "chain with name {:?} already exists!", self.name);

        fp.create_directory();
        let s = ResourcesSubDirectory {
            name: self.name.clone(),
            root_directory: r.directory.clone(),
            directory: fp.clone(),
        };

        let json_fp = fp.clone().append("creator_module/module.json");
        json_fp.save_object_to_json_file(self);
        let ron_fp =  fp.clone().append("creator_module/module.ron");
        ron_fp.save_object_to_ron_file(self);
        let yaml_fp =  fp.clone().append("creator_module/module.yaml");
        yaml_fp.save_object_to_yaml_file(self);

        s.preprocess_environment(force_build_on_all);
    }
    pub fn load(r: &ResourcesRootDirectory, name: &str) -> Self {
        let s = r.get_subdirectory(name);
        let fp = s.directory.clone().append("creator_module/module.json");
        return fp.load_object_from_json_file()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ChainCreatorAction {
    AddAlreadyExistingChain { name: String, base_offset: ISE3q, scale: [f64; 3] },
    AddSingleLinkFromStlFile {
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        object_name: String,
        parent_object: Option<String>,
        base_offset: ISE3q,
        scale: [f64; 3]
    },
    AddSingleLinkFromObjFile {
        // #[serde(skip_serializing, default)] fp: PathBuf,
        fp: PathBuf,
        object_name: String,
        parent_object: Option<String>,
        base_offset: ISE3q,
        scale: [f64; 3]
    },
    AddSingleLinkFromGlbFile {
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
impl ChainCreatorAction {
    pub fn to_signature(&self) -> ChainCreatorActionSignature {
        match self {
            ChainCreatorAction::AddAlreadyExistingChain { name, .. } => { ChainCreatorActionSignature::AddAlreadyExistingEnvironment { name: name.clone() } }
            ChainCreatorAction::AddSingleLinkFromStlFile { object_name, .. } => { ChainCreatorActionSignature::AddSingleObjectFromStlFile { object_name: object_name.clone() } }
            ChainCreatorAction::AddSingleLinkFromObjFile { object_name, .. } => { ChainCreatorActionSignature::AddSingleObjectFromObjFile { object_name: object_name.clone() } }
            ChainCreatorAction::AddSingleLinkFromGlbFile { object_name, .. } => { ChainCreatorActionSignature::AddSingleObjectFromGlbFile { object_name: object_name.clone() } }
            ChainCreatorAction::AddSceneFromGlbFile { scene_name, .. } => { ChainCreatorActionSignature::AddSceneFromGlbFile { scene_name: scene_name.to_string() } }
            ChainCreatorAction::SetJointType { parent_object, child_object, .. } => {  ChainCreatorActionSignature::SetJointType { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            ChainCreatorAction::SetJointAxis { parent_object, child_object, .. } => { ChainCreatorActionSignature::SetJointAxis { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            ChainCreatorAction::SetJointLimit { parent_object, child_object, .. } => { ChainCreatorActionSignature::SetJointLimit { parent_object: parent_object.clone(), child_object: child_object.clone() } }
            ChainCreatorAction::SetObjectSimulationMode { object_name, .. } => { ChainCreatorActionSignature::SetObjectSimulationMode { object_name: object_name.clone() } }
            ChainCreatorAction::SetObjectMass { object_name, .. } => { ChainCreatorActionSignature::SetObjectMass { object_name: object_name.clone() } }
            ChainCreatorAction::SetObjectInertialOrigin { object_name, .. } => { ChainCreatorActionSignature::SetObjectInertialOrigin { object_name: object_name.clone() } }
            ChainCreatorAction::SetObjectInertia { object_name, .. } => { ChainCreatorActionSignature::SetObjectInertia { object_name: object_name.clone() } }
        }
    }

    pub fn is_matching(&self, other: &ChainCreatorAction) -> bool {
        return self.to_signature() == other.to_signature()
    }

    pub fn is_matching_signature(&self, signature: &ChainCreatorActionSignature) -> bool {
        &self.to_signature() == signature
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub enum ChainCreatorActionSignature {
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
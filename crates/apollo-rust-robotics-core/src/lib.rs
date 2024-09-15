use parry3d_f64::query::Contact;
use apollo_rust_linalg::V;
use apollo_rust_proximity::double_group_queries::{DoubleGroupProximityQueryMode, DoubleGroupProximityQueryOutput};
use apollo_rust_proximity::proxima::proxima1::{Proxima1, Proxima1Cache};
use apollo_rust_proximity::proxima::proxima2::{LieAlgMode, Proxima2, Proxima2Cache};
use apollo_rust_proximity::proxima::proxima_core::{ProximaBudget, ProximaOutput, ProximaTrait};
use apollo_rust_proximity::ProximityLossFunction;
use apollo_rust_modules::{ResourcesSubDirectory};
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_proximity::bvh::{Bvh, BvhShape};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::LinkShapesMaxDistanceFromOriginTrait;
use crate::modules_runtime::link_shapes_distance_statistics_nalgebra_module::ApolloLinkShapesDistanceStatisticsNalgebraModule;
use crate::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};
use crate::modules_runtime::link_shapes_simple_skips_nalgebra_module::ApolloLinkShapesSimpleSkipsNalgebraModule;
use crate::modules_runtime::link_shapes_skips_nalgebra_module::ApolloLinkShapesSkipsNalgebraModule;
use crate::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use crate::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use crate::robot_functions::robot_proximity_functions::RobotProximityFunctions;

/// The apollo-rust-robotics-core module contains robotics functions and structs that depend
/// only on robot modules, but without initializing them.  Structs in this crate are initialized
/// using separate traits in the apollo-rust-robotics crate.  In practice, you should be using
/// that crate for all robotics needs.  This crate structure helps avoid circular dependencies.

pub mod modules_runtime;
pub mod robot_functions;
pub mod modules;
/*
#[derive(Clone, Debug)]
pub struct ResourcesRobotsDirectory {
    pub directory: PathBuf
}
impl ResourcesRobotsDirectory {
    pub fn new(directory: PathBuf) -> Self {
        assert!(directory.exists());

        ResourcesRobotsDirectory {
            directory
        }
    }
    pub fn new_default() -> Self {
        Self::new(PathBuf::new_from_default_apollo_robots_dir())
    }
    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }
    pub fn get_robot_subdirectory(&self, robot_name: &str) -> ResourcesSingleRobotDirectory {
        let directory = self.directory.clone().append(robot_name);
        assert!(directory.exists(), "{}", format!("directory {:?} does not exist", directory));
        ResourcesSingleRobotDirectory { robot_name: robot_name.to_string(), robots_directory: self.directory.clone(), directory }
    }
    pub fn get_all_robot_subdirectories(&self) -> Vec<ResourcesSingleRobotDirectory> {
        let mut out = vec![];

        let items = self.directory().get_all_items_in_directory(true, false, false, false);
        items.iter().for_each(|x| {
            let robot_name = x.iter().last().unwrap().to_str().unwrap().to_string();
            out.push(ResourcesSingleRobotDirectory { robot_name, robots_directory: self.directory.clone(), directory: x.clone() })
        });

        out
    }

    /*
    pub fn to_robot(&self, robot_name: &str) -> Robot {
        Robot::new_from_root(self, robot_name)
    }
    */

    /*
    pub fn preprocess_all(&self, force_build_on_all: bool) {
        for x in self.get_all_robot_subdirectories() {
            x.preprocess(force_build_on_all);
        }
    }
    */
}

/*
impl ResourcesRootDirectoryTrait for ResourcesRobotsDirectory {
    type SubDirectoryType = ResourcesSingleRobotDirectory;

    fn new(directory: PathBuf) -> Self {
        Self {
            directory,
        }
    }

    fn new_default() -> Self {
        Self::new(PathBuf::new_from_default_apollo_robots_dir())
    }

    fn directory(&self) -> &PathBuf {
        &self.directory
    }
}
*/

#[derive(Clone, Debug)]
pub struct ResourcesSingleRobotDirectory {
    pub robot_name: String,
    pub robots_directory: PathBuf,
    pub directory: PathBuf
}

impl ResourcesSingleRobotDirectory {
    /*
    pub fn preprocess(&self, force_build_on_all: bool) {
        ApolloURDFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloDOFModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloChainModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConnectionsModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloOriginalMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloPlainMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexHullMeshesModule::load_or_build(self, force_build_on_all).expect("error");
        ApolloConvexDecompositionMeshesModule::load_or_build(self, force_build_on_all).expect("error");
    }
    */
    pub fn robot_name(&self) -> &str {
        &self.robot_name
    }

    pub fn robots_directory(&self) -> &PathBuf {
        &self.robots_directory
    }

    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }
}
*/

/// The `ChainNalgebra` struct represents a kinematic chain and associated modules.
///
/// # Fields
/// - `resources_sub_directory`: A `ResourcesSubDirectory` for managing subdirectories.
/// - `urdf_module`: The URDF module for this chain.
/// - `chain_module`: The chain module containing kinematic chain information.
/// - `dof_module`: The DOF (Degrees of Freedom) module.
/// - `connections_module`: Module that handles connections in the robot.
/// - `original_meshes_module`: Module that handles original meshes.
/// - `plain_meshes_module`: Module that handles plain meshes.
/// - `convex_hull_meshes_module`: Module that handles convex hull meshes.
/// - `convex_decomposition_meshes_module`: Module that handles convex decomposition meshes.
/// - `link_shapes_module`: Module that handles link shapes.
/// - `link_shapes_approximations_module`: Module for link shape approximations.
/// - `link_shapes_max_distance_from_origin_module`: Module for link shape distances from origin.
/// - `link_shapes_distance_statistics_module`: Module for link shape distance statistics.
/// - `link_shapes_simple_skips_nalgebra_module`: Module for handling simple skips in link shapes.
/// - `link_shapes_skips_nalgebra_module`: Module for handling skips in link shapes.
/// - `bounds_module`: Module that handles bounds information.
#[derive(Clone)]
pub struct ChainNalgebra {
    pub resources_sub_directory: ResourcesSubDirectory,
    pub urdf_module: ApolloURDFNalgebraModule,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub connections_module: ApolloConnectionsModule,
    pub original_meshes_module: ApolloOriginalMeshesModule,
    pub plain_meshes_module: ApolloPlainMeshesModule,
    pub convex_hull_meshes_module: ApolloConvexHullMeshesModule,
    pub convex_decomposition_meshes_module: ApolloConvexDecompositionMeshesModule,
    pub link_shapes_module: ApolloLinkShapesModule,
    pub link_shapes_approximations_module: ApolloLinkShapesApproximationsModule,
    pub link_shapes_max_distance_from_origin_module: ApolloLinkShapesMaxDistanceFromOriginModule,
    pub link_shapes_distance_statistics_module: ApolloLinkShapesDistanceStatisticsNalgebraModule,
    pub link_shapes_simple_skips_nalgebra_module: ApolloLinkShapesSimpleSkipsNalgebraModule,
    pub link_shapes_skips_nalgebra_module: ApolloLinkShapesSkipsNalgebraModule,
    // pub link_shapes_lie_alg_error_models_nalgebra_module: ApolloLinkShapesLieAlgErrorModelsNalgebraModule,
    pub bounds_module: ApolloBoundsModule
}
impl ChainNalgebra {
    /// Returns a reference to the resources subdirectory.
    #[inline(always)]
    pub fn resources_sub_directory(&self) -> &ResourcesSubDirectory {
        &self.resources_sub_directory
    }

    /// Returns a reference to the URDF module.
    #[inline(always)]
    pub fn urdf_module(&self) -> &ApolloURDFNalgebraModule {
        &self.urdf_module
    }

    /// Returns a reference to the chain module.
    #[inline(always)]
    pub fn chain_module(&self) -> &ApolloChainModule {
        &self.chain_module
    }

    /// Returns a reference to the DOF module.
    #[inline(always)]
    pub fn dof_module(&self) -> &ApolloDOFModule {
        &self.dof_module
    }

    /// Returns a reference to the connections module.
    #[inline(always)]
    pub fn connections_module(&self) -> &ApolloConnectionsModule {
        &self.connections_module
    }

    /// Returns a reference to the original meshes module.
    #[inline(always)]
    pub fn original_meshes_module(&self) -> &ApolloOriginalMeshesModule {
        &self.original_meshes_module
    }

    /// Returns a reference to the plain meshes module.
    #[inline(always)]
    pub fn plain_meshes_module(&self) -> &ApolloPlainMeshesModule {
        &self.plain_meshes_module
    }

    /// Returns a reference to the convex hull meshes module.
    #[inline(always)]
    pub fn convex_hull_meshes_module(&self) -> &ApolloConvexHullMeshesModule {
        &self.convex_hull_meshes_module
    }

    /// Returns a reference to the convex decomposition meshes module.
    #[inline(always)]
    pub fn convex_decomposition_meshes_module(&self) -> &ApolloConvexDecompositionMeshesModule {
        &self.convex_decomposition_meshes_module
    }

    /// Returns a reference to the link shapes module.
    #[inline(always)]
    pub fn link_shapes_module(&self) -> &ApolloLinkShapesModule {
        &self.link_shapes_module
    }

    /// Returns a reference to the link shapes approximations module.
    #[inline(always)]
    pub fn link_shapes_approximations_module(&self) -> &ApolloLinkShapesApproximationsModule { &self.link_shapes_approximations_module }

    /// Returns a reference to the link shapes max distance from origin module.
    #[inline(always)]
    pub fn link_shapes_max_distance_from_origin_module(&self) -> &ApolloLinkShapesMaxDistanceFromOriginModule {
        &self.link_shapes_max_distance_from_origin_module
    }

    /// Returns a reference to the link shapes distance statistics module.
    #[inline(always)]
    pub fn link_shapes_distance_statistics_module(&self) -> &ApolloLinkShapesDistanceStatisticsNalgebraModule {
        &self.link_shapes_distance_statistics_module
    }

    /// Returns a reference to the link shapes simple skips module.
    #[inline(always)]
    pub fn link_shapes_simple_skips_nalgebra_module(&self) -> &ApolloLinkShapesSimpleSkipsNalgebraModule {
        &self.link_shapes_simple_skips_nalgebra_module
    }

    /// Returns a reference to the link shapes skips module.
    #[inline(always)]
    pub fn link_shapes_skips_nalgebra_module(&self) -> &ApolloLinkShapesSkipsNalgebraModule {
        &self.link_shapes_skips_nalgebra_module
    }

    /// Returns a reference to the bounds module.
    #[inline(always)]
    pub fn bounds_module(&self) -> &ApolloBoundsModule {
        &self.bounds_module
    }
}
impl ChainNalgebra {
    /// Returns the number of degrees of freedom (DOFs) for the chain.
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.dof_module.num_dofs
    }

    /// Computes the forward kinematics (FK) for the robot chain given a state.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    ///
    /// # Returns
    /// A vector of `ISE3q` representing the poses of each link in the chain.
    #[inline]
    pub fn fk(&self, state: &V) -> Vec<ISE3q> {
        RobotKinematicsFunctions::fk(state, self.urdf_module(), self.chain_module(), self.dof_module())
    }

    /// Computes the reverse of forward kinematics, given the link frames.
    ///
    /// # Arguments
    /// - `link_frame`: A reference to a vector of `ISE3q` representing the link frames.
    ///
    /// # Returns
    /// A `V` representing the state that would result in the given link poses.
    #[inline]
    pub fn reverse_of_fk(&self, link_frame: &Vec<ISE3q>) -> V {
        RobotKinematicsFunctions::reverse_of_fk(link_frame, &self.urdf_module, &self.chain_module, &self.dof_module)
    }

    /// Checks for self-intersections of the robot's links.
    ///
    /// # Arguments
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if self-intersections were found.
    pub fn self_intersect(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_intersect(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop)
    }

    /// Checks for self-intersections of the robot's links from a given state.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if self-intersections were found.
    pub fn self_intersect_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let link_poses = self.fk(state);
        self.self_intersect(&link_poses, link_shape_mode, link_shape_rep, early_stop)
    }

    /// Checks for self-intersections using a BVH.
    ///
    /// # Arguments
    /// - `bvh`: A mutable reference to a BVH structure.
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if self-intersections were found.
    pub fn self_intersect_bvh<B: BvhShape>(&self, bvh: &mut Bvh<B>, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_intersect_bvh(bvh, self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop)
    }

    /// Computes the self-distance between links in the chain.
    ///
    /// # Arguments
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<f64>` representing the distance.
    pub fn self_distance(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_distance(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop)
    }

    /// Computes the self-distance from a given state.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<f64>` representing the distance.
    pub fn self_distance_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let link_poses = self.fk(state);
        self.self_distance(&link_poses, link_shape_mode, link_shape_rep, early_stop)
    }

    /// Computes contact points between links in the chain.
    ///
    /// # Arguments
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn self_contact(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_contact(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop, margin)
    }

    /// Computes contact points from a given state.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn self_contact_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let link_poses = self.fk(state);
        self.self_contact(&link_poses, link_shape_mode, link_shape_rep, early_stop, margin)
    }

    /// Computes self-contact using a BVH.
    ///
    /// # Arguments
    /// - `bvh`: A mutable reference to a BVH structure.
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn self_contact_bvh<B: BvhShape>(&self, bvh: &mut Bvh<B>, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_contact_bvh(bvh, self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop, margin)
    }

    /// Checks for double-chain intersections between this chain and another chain.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if intersections were found.
    pub fn double_chain_intersect(&self,
                                  other_chain: &ChainNalgebra,
                                  self_link_poses: &Vec<ISE3q>,
                                  self_link_shape_mode: LinkShapeMode,
                                  self_link_shape_rep: LinkShapeRep,
                                  other_link_poses: &Vec<ISE3q>,
                                  other_link_shape_mode: LinkShapeMode,
                                  other_link_shape_rep: LinkShapeRep,
                                  early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        RobotProximityFunctions::double_chain_intersect(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop, &DoubleGroupProximityQueryMode::AllPossiblePairs)
    }

    /// Checks for double-chain intersections using BVHs.
    ///
    /// # Arguments
    /// - `self_bvh`: A mutable reference to a BVH structure for this chain.
    /// - `other_bvh`: A mutable reference to a BVH structure for the other chain.
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if intersections were found.
    pub fn double_chain_intersect_bvh<B: BvhShape>(&self,
                                                   self_bvh: &mut Bvh<B>,
                                                   other_bvh: &mut Bvh<B>,
                                                   other_chain: &ChainNalgebra,
                                                   self_link_poses: &Vec<ISE3q>,
                                                   self_link_shape_mode: LinkShapeMode,
                                                   self_link_shape_rep: LinkShapeRep,
                                                   other_link_poses: &Vec<ISE3q>,
                                                   other_link_shape_mode: LinkShapeMode,
                                                   other_link_shape_rep: LinkShapeRep,
                                                   early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        RobotProximityFunctions::double_chain_intersect_bvh(self_bvh, other_bvh, &self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop)
    }

    /// Checks for double-chain intersections from states.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_state`: A reference to a `V` representing the robot state for this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_state`: A reference to a `V` representing the robot state for the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<bool>` indicating if intersections were found.
    pub fn double_chain_intersect_from_states(&self,
                                              other_chain: &ChainNalgebra,
                                              self_state: &V,
                                              self_link_shape_mode: LinkShapeMode,
                                              self_link_shape_rep: LinkShapeRep,
                                              other_state: &V,
                                              other_link_shape_mode: LinkShapeMode,
                                              other_link_shape_rep: LinkShapeRep,
                                              early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let self_link_poses = self.fk(self_state);
        let other_link_poses = other_chain.fk(other_state);

        self.double_chain_intersect(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop)
    }

    /// Computes the distance between two robot chains.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<f64>` representing the distance between the chains.
    pub fn double_chain_distance(&self,
                                 other_chain: &ChainNalgebra,
                                 self_link_poses: &Vec<ISE3q>,
                                 self_link_shape_mode: LinkShapeMode,
                                 self_link_shape_rep: LinkShapeRep,
                                 other_link_poses: &Vec<ISE3q>,
                                 other_link_shape_mode: LinkShapeMode,
                                 other_link_shape_rep: LinkShapeRep,
                                 early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        RobotProximityFunctions::double_chain_distance(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop, &DoubleGroupProximityQueryMode::AllPossiblePairs)
    }

    /// Computes the distance between two robot chains from states.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_state`: A reference to a `V` representing the robot state for this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_state`: A reference to a `V` representing the robot state for the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<f64>` representing the distance between the chains.
    pub fn double_chain_distance_from_states(&self,
                                             other_chain: &ChainNalgebra,
                                             self_state: &V,
                                             self_link_shape_mode: LinkShapeMode,
                                             self_link_shape_rep: LinkShapeRep,
                                             other_state: &V,
                                             other_link_shape_mode: LinkShapeMode,
                                             other_link_shape_rep: LinkShapeRep,
                                             early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let self_link_poses = self.fk(self_state);
        let other_link_poses = other_chain.fk(other_state);

        self.double_chain_distance(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop)
    }

    /// Computes the contact points between two robot chains.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn double_chain_contact(&self,
                                other_chain: &ChainNalgebra,
                                self_link_poses: &Vec<ISE3q>,
                                self_link_shape_mode: LinkShapeMode,
                                self_link_shape_rep: LinkShapeRep,
                                other_link_poses: &Vec<ISE3q>,
                                other_link_shape_mode: LinkShapeMode,
                                other_link_shape_rep: LinkShapeRep,
                                early_stop: bool,
                                margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        RobotProximityFunctions::double_chain_contact(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop, margin, &DoubleGroupProximityQueryMode::AllPossiblePairs)
    }

    /// Computes the contact points between two robot chains from states.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_state`: A reference to a `V` representing the robot state for this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_state`: A reference to a `V` representing the robot state for the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn double_chain_contact_from_states(&self,
                                            other_chain: &ChainNalgebra,
                                            self_state: &V,
                                            self_link_shape_mode: LinkShapeMode,
                                            self_link_shape_rep: LinkShapeRep,
                                            other_state: &V,
                                            other_link_shape_mode: LinkShapeMode,
                                            other_link_shape_rep: LinkShapeRep,
                                            early_stop: bool,
                                            margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let self_link_poses = self.fk(self_state);
        let other_link_poses = other_chain.fk(other_state);

        self.double_chain_contact(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop, margin)
    }

    /// Computes the contact points between two robot chains using BVHs.
    ///
    /// # Arguments
    /// - `self_bvh`: A mutable reference to a BVH structure for this chain.
    /// - `other_bvh`: A mutable reference to a BVH structure for the other chain.
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `early_stop`: A boolean flag to enable early stopping during checks.
    /// - `margin`: A margin value for the proximity calculation.
    ///
    /// # Returns
    /// A `DoubleGroupProximityQueryOutput<Option<Contact>>` representing the contact points.
    pub fn double_chain_contact_bvh<B: BvhShape>(&self,
                                                 self_bvh: &mut Bvh<B>,
                                                 other_bvh: &mut Bvh<B>,
                                                 other_chain: &ChainNalgebra,
                                                 self_link_poses: &Vec<ISE3q>,
                                                 self_link_shape_mode: LinkShapeMode,
                                                 self_link_shape_rep: LinkShapeRep,
                                                 other_link_poses: &Vec<ISE3q>,
                                                 other_link_shape_mode: LinkShapeMode,
                                                 other_link_shape_rep: LinkShapeRep,
                                                 early_stop: bool,
                                                 margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        RobotProximityFunctions::double_chain_contact_bvh(self_bvh, other_bvh, &self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop, margin)
    }

    /// Computes self-intersections using Proxima.
    ///
    /// # Arguments
    /// - `proxima`: A mutable reference to a Proxima instance.
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `frozen`: A boolean flag indicating whether the Proxima process should be frozen.
    ///
    /// # Returns
    /// A `ProximaOutput<bool>` representing if intersections were found.
    pub fn self_intersect_proxima<P: ProximaTrait>(&self, proxima: &mut P, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, frozen: bool) -> ProximaOutput<bool> {
        let link_shapes_module = &self.link_shapes_module;
        let skips = self.link_shapes_skips_nalgebra_module.get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_intersect_proxima(proxima, link_shapes_module, link_poses, link_shape_mode, link_shape_rep, Some(skips), frozen)
    }

    /// Computes the proximity values using Proxima for a single chain.
    ///
    /// # Arguments
    /// - `proxima`: A mutable reference to a Proxima instance.
    /// - `budget`: A reference to the proximity budget.
    /// - `loss_function`: A reference to the proximity loss function.
    /// - `p_norm`: A floating-point value representing the p-norm used in the loss function.
    /// - `cutoff_distance`: A floating-point value representing the cutoff distance for proximity calculations.
    /// - `link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the chain.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `frozen`: A boolean flag indicating whether the Proxima process should be frozen.
    ///
    /// # Returns
    /// A `ProximaOutput<f64>` representing the proximity values.
    pub fn self_proximity_proxima<P: ProximaTrait>(&self, proxima: &mut P, budget: &ProximaBudget, loss_function: &ProximityLossFunction, p_norm: f64, cutoff_distance: f64, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, frozen: bool) -> ProximaOutput<f64> {
        let link_shapes_module = &self.link_shapes_module;
        let skips = self.link_shapes_skips_nalgebra_module.get_skips(link_shape_mode, link_shape_rep);
        let average_distances = &self.link_shapes_distance_statistics_module.get_stats(&link_shape_rep, &link_shape_mode).averages;
        RobotProximityFunctions::self_proximity_proxima(proxima, budget, loss_function, p_norm, cutoff_distance, link_shapes_module, link_poses, link_shape_mode, link_shape_rep, Some(skips), Some(average_distances), frozen)
    }


    /// Checks for double-chain intersections using Proxima.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `proxima`: A mutable reference to a Proxima instance.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `frozen`: A boolean flag indicating whether the Proxima process should be frozen.
    ///
    /// # Returns
    /// A `ProximaOutput<bool>` indicating if intersections were found.
    pub fn double_chain_intersect_proxima<P: ProximaTrait>(&self, other_chain: &ChainNalgebra, proxima: &mut P, self_link_poses: &Vec<ISE3q>, self_link_shape_mode: LinkShapeMode, self_link_shape_rep: LinkShapeRep, other_link_poses: &Vec<ISE3q>, other_link_shape_mode: LinkShapeMode, other_link_shape_rep: LinkShapeRep, frozen: bool) -> ProximaOutput<bool> {
        let self_link_shapes_module = &self.link_shapes_module;
        let other_link_shapes_module = &other_chain.link_shapes_module;
        RobotProximityFunctions::double_chain_intersect_proxima(proxima, self_link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, other_link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, frozen)
    }

    /// Computes the proximity values using Proxima for double chains.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `proxima`: A mutable reference to a Proxima instance.
    /// - `budget`: A reference to the proximity budget.
    /// - `loss_function`: A reference to the proximity loss function.
    /// - `p_norm`: A floating-point value representing the p-norm used in the loss function.
    /// - `cutoff_distance`: A floating-point value representing the cutoff distance for proximity calculations.
    /// - `self_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_link_poses`: A reference to a vector of `ISE3q` representing the poses of each link in the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `frozen`: A boolean flag indicating whether the Proxima process should be frozen.
    ///
    /// # Returns
    /// A `ProximaOutput<f64>` representing the proximity values.
    pub fn double_chain_proximity_proxima<P: ProximaTrait>(&self,
                                                           other_chain: &ChainNalgebra,
                                                           proxima: &mut P,
                                                           budget: &ProximaBudget,
                                                           loss_function: &ProximityLossFunction,
                                                           p_norm: f64,
                                                           cutoff_distance: f64,
                                                           self_link_poses: &Vec<ISE3q>,
                                                           self_link_shape_mode: LinkShapeMode,
                                                           self_link_shape_rep: LinkShapeRep,
                                                           other_link_poses: &Vec<ISE3q>,
                                                           other_link_shape_mode: LinkShapeMode,
                                                           other_link_shape_rep: LinkShapeRep,
                                                           frozen: bool) -> ProximaOutput<f64> {
        let self_link_shapes_module = &self.link_shapes_module;
        let other_link_shapes_module = &other_chain.link_shapes_module;
        RobotProximityFunctions::double_chain_proximity_proxima(proxima, budget, loss_function, p_norm, cutoff_distance, self_link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, other_link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, frozen)
    }

    /// Retrieves a Proxima1 instance for this chain.
    ///
    /// # Arguments
    /// - `interpolation`: A floating-point value representing the interpolation factor.
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    ///
    /// # Returns
    /// A `Proxima1` instance.
    pub fn get_self_proxima1(&self, interpolation: f64, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep) -> Proxima1 {
        let fk_res = self.fk(state);
        let shapes = self.link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = self.link_shapes_module.link_poses_to_shape_poses(&fk_res, link_shape_mode);
        let skips = self.link_shapes_skips_nalgebra_module.get_skips(link_shape_mode, link_shape_rep);
        let cache = Proxima1Cache::new(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, Some(skips));

        Proxima1 {
            cache,
            max_distances_from_origin_a: self.link_shapes_max_distance_from_origin_module.get_shapes_max_distances_from_origin(link_shape_mode, link_shape_rep),
            max_distances_from_origin_b: self.link_shapes_max_distance_from_origin_module.get_shapes_max_distances_from_origin(link_shape_mode, link_shape_rep),
            interpolation,
        }
    }

    /// Retrieves a Proxima1 instance for double chains.
    ///
    /// # Arguments
    /// - `interpolation`: A floating-point value representing the interpolation factor.
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_state`: A reference to a `V` representing the robot state for this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_state`: A reference to a `V` representing the robot state for the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    ///
    /// # Returns
    /// A `Proxima1` instance.
    pub fn get_double_chain_proxima1(&self, interpolation: f64, other_chain: &ChainNalgebra, self_state: &V, self_link_shape_mode: LinkShapeMode, self_link_shape_rep: LinkShapeRep, other_state: &V, other_link_shape_mode: LinkShapeMode, other_link_shape_rep: LinkShapeRep) -> Proxima1 {
        let self_fk_res = self.fk(self_state);
        let self_shapes = self.link_shapes_module.get_shapes(self_link_shape_mode, self_link_shape_rep);
        let self_poses = self.link_shapes_module.link_poses_to_shape_poses(&self_fk_res, self_link_shape_mode);

        let other_fk_res = other_chain.fk(other_state);
        let other_shapes = other_chain.link_shapes_module.get_shapes(other_link_shape_mode, other_link_shape_rep);
        let other_poses = self.link_shapes_module.link_poses_to_shape_poses(&other_fk_res, other_link_shape_mode);
        let cache = Proxima1Cache::new(self_shapes, &self_poses, other_shapes, &other_poses, &DoubleGroupProximityQueryMode::AllPossiblePairs, None);

        Proxima1 {
            cache,
            max_distances_from_origin_a: self.link_shapes_max_distance_from_origin_module.get_shapes_max_distances_from_origin(self_link_shape_mode, self_link_shape_rep),
            max_distances_from_origin_b: other_chain.link_shapes_max_distance_from_origin_module.get_shapes_max_distances_from_origin(other_link_shape_mode, other_link_shape_rep),
            interpolation,
        }
    }

    /// Retrieves a Proxima2 instance for this chain.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `lie_alg_mode`: The Lie algebra mode.
    ///
    /// # Returns
    /// A `Proxima2` instance.
    pub fn get_self_proxima2(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, lie_alg_mode: LieAlgMode) -> Proxima2 {
        let fk_res = self.fk(state);
        let shapes = self.link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = self.link_shapes_module.link_poses_to_shape_poses(&fk_res, link_shape_mode);
        let skips = self.link_shapes_skips_nalgebra_module.get_skips(link_shape_mode, link_shape_rep);
        let cache = Proxima2Cache::new(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, Some(skips), lie_alg_mode);

        Proxima2 {
            cache,
            lie_alg_mode,
        }
    }

    /// Retrieves a Proxima2 instance for double chains.
    ///
    /// # Arguments
    /// - `other_chain`: A reference to another `ChainNalgebra`.
    /// - `self_state`: A reference to a `V` representing the robot state for this chain.
    /// - `self_link_shape_mode`: The shape mode of the links in this chain.
    /// - `self_link_shape_rep`: The representation mode of the links in this chain.
    /// - `other_state`: A reference to a `V` representing the robot state for the other chain.
    /// - `other_link_shape_mode`: The shape mode of the links in the other chain.
    /// - `other_link_shape_rep`: The representation mode of the links in the other chain.
    /// - `lie_alg_mode`: The Lie algebra mode.
    ///
    /// # Returns
    /// A `Proxima2` instance.
    pub fn get_double_chain_proxima2(&self, other_chain: &ChainNalgebra, self_state: &V, self_link_shape_mode: LinkShapeMode, self_link_shape_rep: LinkShapeRep, other_state: &V, other_link_shape_mode: LinkShapeMode, other_link_shape_rep: LinkShapeRep, lie_alg_mode: LieAlgMode) -> Proxima2 {
        let self_fk_res = self.fk(self_state);
        let self_shapes = self.link_shapes_module.get_shapes(self_link_shape_mode, self_link_shape_rep);
        let self_poses = self.link_shapes_module.link_poses_to_shape_poses(&self_fk_res, self_link_shape_mode);

        let other_fk_res = other_chain.fk(other_state);
        let other_shapes = other_chain.link_shapes_module.get_shapes(other_link_shape_mode, other_link_shape_rep);
        let other_poses = self.link_shapes_module.link_poses_to_shape_poses(&other_fk_res, other_link_shape_mode);

        let cache = Proxima2Cache::new(self_shapes, &self_poses, other_shapes, &other_poses, &DoubleGroupProximityQueryMode::AllPossiblePairs, None, lie_alg_mode);

        Proxima2 {
            cache,
            lie_alg_mode,
        }
    }

    /// Builds and returns a BVH (Bounding Volume Hierarchy) for this chain.
    ///
    /// # Arguments
    /// - `state`: A reference to a `V` representing the robot state.
    /// - `link_shape_mode`: The shape mode of the links.
    /// - `link_shape_rep`: The representation mode of the links.
    /// - `branch_factor`: The branching factor of the BVH.
    ///
    /// # Returns
    /// A `Bvh<B>` instance representing the built BVH.
    pub fn get_bvh<B: BvhShape>(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, branch_factor: usize) -> Bvh<B> {
        let fk_res = self.fk(state);
        let shapes = self.link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = self.link_shapes_module.link_poses_to_shape_poses(&fk_res, link_shape_mode);

        Bvh::build(shapes, &poses, branch_factor)
    }
}

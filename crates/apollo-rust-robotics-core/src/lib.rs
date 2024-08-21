use parry3d_f64::query::Contact;
use apollo_rust_linalg::V;
use apollo_rust_proximity::double_group_queries::{DoubleGroupProximityQueryMode, DoubleGroupProximityQueryOutput};
use apollo_rust_robot_modules::{ResourcesSubDirectory};
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
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


#[derive(Clone)]
pub struct ChainNalgebra {
    pub single_robot_directory: ResourcesSubDirectory,
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
    pub bounds_module: ApolloBoundsModule
}
impl ChainNalgebra {
    #[inline(always)]
    pub fn resources_sub_directory(&self) -> &ResourcesSubDirectory {
        &self.single_robot_directory
    }

    #[inline(always)]
    pub fn urdf_module(&self) -> &ApolloURDFNalgebraModule {
        &self.urdf_module
    }

    #[inline(always)]
    pub fn chain_module(&self) -> &ApolloChainModule {
        &self.chain_module
    }

    #[inline(always)]
    pub fn dof_module(&self) -> &ApolloDOFModule {
        &self.dof_module
    }

    #[inline(always)]
    pub fn connections_module(&self) -> &ApolloConnectionsModule {
        &self.connections_module
    }

    #[inline(always)]
    pub fn original_meshes_module(&self) -> &ApolloOriginalMeshesModule {
        &self.original_meshes_module
    }

    #[inline(always)]
    pub fn plain_meshes_module(&self) -> &ApolloPlainMeshesModule {
        &self.plain_meshes_module
    }

    #[inline(always)]
    pub fn convex_hull_meshes_module(&self) -> &ApolloConvexHullMeshesModule {
        &self.convex_hull_meshes_module
    }

    #[inline(always)]
    pub fn convex_decomposition_meshes_module(&self) -> &ApolloConvexDecompositionMeshesModule {
        &self.convex_decomposition_meshes_module
    }

    #[inline(always)]
    pub fn link_shapes_module(&self) -> &ApolloLinkShapesModule {
        &self.link_shapes_module
    }

    #[inline(always)]
    pub fn link_shapes_approximations_module(&self) -> &ApolloLinkShapesApproximationsModule { &self.link_shapes_approximations_module }

    #[inline(always)]
    pub fn link_shapes_max_distance_from_origin_module(&self) -> &ApolloLinkShapesMaxDistanceFromOriginModule {
        &self.link_shapes_max_distance_from_origin_module
    }

    #[inline(always)]
    pub fn link_shapes_distance_statistics_module(&self) -> &ApolloLinkShapesDistanceStatisticsNalgebraModule {
        &self.link_shapes_distance_statistics_module
    }

    #[inline(always)]
    pub fn link_shapes_simple_skips_nalgebra_module(&self) -> &ApolloLinkShapesSimpleSkipsNalgebraModule {
        &self.link_shapes_simple_skips_nalgebra_module
    }

    #[inline(always)]
    pub fn link_shapes_skips_nalgebra_module(&self) -> &ApolloLinkShapesSkipsNalgebraModule {
        &self.link_shapes_skips_nalgebra_module
    }

    #[inline(always)]
    pub fn bounds_module(&self) -> &ApolloBoundsModule {
        &self.bounds_module
    }
}
impl ChainNalgebra {
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.dof_module.num_dofs
    }

    #[inline]
    pub fn fk(&self, state: &V) -> Vec<ISE3q> {
        RobotKinematicsFunctions::fk(state, self.urdf_module(), self.chain_module(), self.dof_module())
    }

    #[inline]
    pub fn reverse_of_fk(&self, link_frame: &Vec<ISE3q>) -> V {
        RobotKinematicsFunctions::reverse_of_fk(link_frame, &self.urdf_module, &self.chain_module, &self.dof_module)
    }

    pub fn self_intersect(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_intersect(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop)
    }

    pub fn self_intersect_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let link_poses = self.fk(state);
        self.self_intersect(&link_poses, link_shape_mode, link_shape_rep, early_stop)
    }

    pub fn self_distance(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_distance(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop)
    }

    pub fn self_distance_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let link_poses = self.fk(state);
        self.self_distance(&link_poses, link_shape_mode, link_shape_rep, early_stop)
    }

    pub fn self_contact(&self, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let skips = self.link_shapes_simple_skips_nalgebra_module().get_skips(link_shape_mode, link_shape_rep);
        RobotProximityFunctions::self_contact(self.link_shapes_module(), link_poses, link_shape_mode, link_shape_rep, Some(skips), early_stop, margin)
    }

    pub fn self_contact_from_state(&self, state: &V, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let link_poses = self.fk(state);
        self.self_contact(&link_poses, link_shape_mode, link_shape_rep, early_stop, margin)
    }

    pub fn other_chain_intersect(&self,
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

    pub fn other_chain_intersect_from_states(&self,
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

        self.other_chain_intersect(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop)
    }

    pub fn other_chain_distance(&self,
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

    pub fn other_chain_distance_from_states(&self,
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

        self.other_chain_distance(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop)
    }

    pub fn other_chain_contact(&self,
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

    pub fn other_chain_contact_from_states(&self,
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

        self.other_chain_contact(other_chain, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_link_poses, other_link_shape_mode, other_link_shape_rep, early_stop, margin)
    }
}

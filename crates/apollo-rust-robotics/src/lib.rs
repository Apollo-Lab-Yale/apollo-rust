use parry3d_f64::query::Contact;
use apollo_rust_linalg::V;
use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;
use apollo_rust_robotics_core::robot_functions::robot_proximity_functions::RobotProximityFunctions;
use apollo_rust_robotics_core::modules_runtime::link_shapes_simple_skips_nalgebra_module::ApolloLinkShapesSimpleSkipsNalgebraModule;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_preprocessor::{PreprocessorModule};
use apollo_rust_proximity::double_group_queries::DoubleGroupProximityQueryOutput;
use apollo_rust_robot_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;

#[derive(Clone)]
pub struct Chain {
    single_robot_directory: ResourcesSubDirectory,
    urdf_module: ApolloURDFNalgebraModule,
    chain_module: ApolloChainModule,
    dof_module: ApolloDOFModule,
    connections_module: ApolloConnectionsModule,
    original_meshes_module: ApolloOriginalMeshesModule,
    plain_meshes_module: ApolloPlainMeshesModule,
    convex_hull_meshes_module: ApolloConvexHullMeshesModule,
    convex_decomposition_meshes_module: ApolloConvexDecompositionMeshesModule,
    link_shapes_module: ApolloLinkShapesModule,
    link_shapes_max_distance_from_origin_module: ApolloLinkShapesMaxDistanceFromOriginModule,
    link_shapes_distance_statistics_module: ApolloLinkShapesDistanceStatisticsModule,
    link_shapes_simple_skips_nalgebra_module: ApolloLinkShapesSimpleSkipsNalgebraModule,
    bounds_module: ApolloBoundsModule
}
impl Chain {
    pub fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self {
        let s = root.get_subdirectory(robot_name);

        Self::new_from_sub_directory(&s)
    }

    pub fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self {
        let urdf_module = ApolloURDFNalgebraModule::from_urdf_module(&ApolloURDFModule::load_or_build(&s, false).expect("error"));
        let chain_module = ApolloChainModule::load_or_build(&s, false).expect("error");
        let dof_module = ApolloDOFModule::load_or_build(&s, false).expect("error");
        let connections_module = ApolloConnectionsModule::load_or_build(&s, false).expect("error");
        let original_meshes_module = ApolloOriginalMeshesModule::load_or_build(&s, false).expect("error");
        let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(&s, false).expect("error");
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(&s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(&s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(&s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);
        let link_shapes_max_distance_from_origin_module = ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(&s, false).expect("error");
        let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsModule::load_or_build(&s, false).expect("error");
        let link_shapes_simple_skips_module = ApolloLinkShapesSimpleSkipsModule::load_or_build(&s, false).expect("error");
        let link_shapes_simple_skips_nalgebra_module = ApolloLinkShapesSimpleSkipsNalgebraModule::from_link_shapes_simple_skips_module(&link_shapes_simple_skips_module);
        let bounds_module = ApolloBoundsModule::load_or_build(&s, false).expect("error");

        Self {
            single_robot_directory: s.clone(),
            urdf_module,
            chain_module,
            dof_module,
            connections_module,
            original_meshes_module,
            plain_meshes_module,
            convex_hull_meshes_module,
            convex_decomposition_meshes_module,
            link_shapes_module,
            link_shapes_max_distance_from_origin_module,
            link_shapes_distance_statistics_module,
            link_shapes_simple_skips_nalgebra_module,
            bounds_module,
        }
    }

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
    pub fn link_shapes_max_distance_from_origin_module(&self) -> &ApolloLinkShapesMaxDistanceFromOriginModule {
        &self.link_shapes_max_distance_from_origin_module
    }

    #[inline(always)]
    pub fn link_shapes_distance_statistics_module(&self) -> &ApolloLinkShapesDistanceStatisticsModule {
        &self.link_shapes_distance_statistics_module
    }

    #[inline(always)]
    pub fn link_shapes_simple_skips_nalgebra_module(&self) -> &ApolloLinkShapesSimpleSkipsNalgebraModule {
        &self.link_shapes_simple_skips_nalgebra_module
    }

    #[inline(always)]
    pub fn bounds_module(&self) -> &ApolloBoundsModule {
        &self.bounds_module
    }
}
impl Chain {
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
                                 other_chain: &Chain,
                                 self_link_poses: &Vec<ISE3q>,
                                 self_link_shape_mode: LinkShapeMode,
                                 self_link_shape_rep: LinkShapeRep,
                                 other_link_poses: &Vec<ISE3q>,
                                 other_link_shape_mode: LinkShapeMode,
                                 other_link_shape_rep: LinkShapeRep,
                                 early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        RobotProximityFunctions::double_chain_intersect(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop)
    }

    pub fn other_chain_intersect_from_states(&self,
                                             other_chain: &Chain,
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
                                 other_chain: &Chain,
                                 self_link_poses: &Vec<ISE3q>,
                                 self_link_shape_mode: LinkShapeMode,
                                 self_link_shape_rep: LinkShapeRep,
                                 other_link_poses: &Vec<ISE3q>,
                                 other_link_shape_mode: LinkShapeMode,
                                 other_link_shape_rep: LinkShapeRep,
                                 early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        RobotProximityFunctions::double_chain_distance(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop)
    }

    pub fn other_chain_distance_from_states(&self,
                                             other_chain: &Chain,
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
                               other_chain: &Chain,
                               self_link_poses: &Vec<ISE3q>,
                               self_link_shape_mode: LinkShapeMode,
                               self_link_shape_rep: LinkShapeRep,
                               other_link_poses: &Vec<ISE3q>,
                               other_link_shape_mode: LinkShapeMode,
                               other_link_shape_rep: LinkShapeRep,
                               early_stop: bool,
                               margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        RobotProximityFunctions::double_chain_contact(&self.link_shapes_module, &self_link_poses, self_link_shape_mode, self_link_shape_rep, &other_chain.link_shapes_module, other_link_poses, other_link_shape_mode, other_link_shape_rep, None, early_stop, margin)
    }

    pub fn other_chain_contact_from_states(&self,
                                           other_chain: &Chain,
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


pub trait ToChain {
    fn to_chain(&self) -> Chain;
}
impl ToChain for ResourcesSubDirectory {
    fn to_chain(&self) -> Chain {
        Chain::new_from_sub_directory(self)
    }
}

pub trait ToChainFromName {
    fn to_chain(&self, robot_name: &str) -> Chain;
}
impl ToChainFromName for ResourcesRootDirectory {
    fn to_chain(&self, robot_name: &str) -> Chain {
        Chain::new_from_root_directory(self, robot_name)
    }
}

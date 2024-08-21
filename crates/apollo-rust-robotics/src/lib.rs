use apollo_rust_robot_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_robot_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_robot_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_robot_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_robot_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{ApolloLinkShapesModule};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_simple_skips_nalgebra_module::ApolloLinkShapesSimpleSkipsNalgebraModule;
use apollo_rust_preprocessor::{PreprocessorModule};
use apollo_rust_robot_modules::{ResourcesRootDirectory, ResourcesSubDirectory};
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_robot_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_robot_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_distance_statistics_nalgebra_module::ApolloLinkShapesDistanceStatisticsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_skips_nalgebra_module::ApolloLinkShapesSkipsNalgebraModule;


pub trait ChainBuildersTrait {
    fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self;

    fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self;
}
impl ChainBuildersTrait for ChainNalgebra {
    fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self {
        let s = root.get_subdirectory(robot_name);

        Self::new_from_sub_directory(&s)
    }

    fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self {
        let urdf_module = ApolloURDFNalgebraModule::from_urdf_module(&ApolloURDFModule::load_or_build(&s, false).expect("error"));
        let chain_module = ApolloChainModule::load_or_build(&s, false).expect("error");
        let dof_module = ApolloDOFModule::load_or_build(&s, false).expect("error");
        let connections_module = ApolloConnectionsModule::load_or_build(&s, false).expect("error");
        let original_meshes_module = ApolloOriginalMeshesModule::load_or_build(&s, false).expect("error");
        let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(&s, false).expect("error");
        let convex_hull_meshes_module = ApolloConvexHullMeshesModule::load_or_build(&s, false).expect("error");
        let convex_decomposition_meshes_module = ApolloConvexDecompositionMeshesModule::load_or_build(&s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(&s, &convex_hull_meshes_module, &convex_decomposition_meshes_module);
        let link_shapes_approximations_module = ApolloLinkShapesApproximationsModule::load_or_build(&s, false).expect("error");
        let link_shapes_max_distance_from_origin_module = ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(&s, false).expect("error");
        let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsModule::load_or_build(&s, false).expect("error");
        let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsNalgebraModule::from_link_shapes_distance_statistics_module(&link_shapes_distance_statistics_module);
        let link_shapes_simple_skips_module = ApolloLinkShapesSimpleSkipsModule::load_or_build(&s, false).expect("error");
        let link_shapes_simple_skips_nalgebra_module = ApolloLinkShapesSimpleSkipsNalgebraModule::from_link_shapes_simple_skips_module(&link_shapes_simple_skips_module);
        let link_shapes_skips_module = ApolloLinkShapesSkipsModule::load_or_build(&s, false).expect("error");
        let link_shapes_skips_nalgebra_module = ApolloLinkShapesSkipsNalgebraModule::from_link_shapes_skips_module(&link_shapes_skips_module);
        let bounds_module = ApolloBoundsModule::load_or_build(&s, false).expect("error");

        Self {
            resources_sub_directory: s.clone(),
            urdf_module,
            chain_module,
            dof_module,
            connections_module,
            original_meshes_module,
            plain_meshes_module,
            convex_hull_meshes_module,
            convex_decomposition_meshes_module,
            link_shapes_module,
            link_shapes_approximations_module,
            link_shapes_max_distance_from_origin_module,
            link_shapes_distance_statistics_module,
            link_shapes_simple_skips_nalgebra_module,
            link_shapes_skips_nalgebra_module,
            bounds_module,
        }
    }
}

pub trait PreprocessForceBuildModulesTrait {
    fn refine_link_shapes_skips_module(self) -> Self;
}
impl PreprocessForceBuildModulesTrait for ChainNalgebra {
    fn refine_link_shapes_skips_module(self) -> Self {
        let s = &self.resources_sub_directory;
        ApolloLinkShapesSkipsModule::load_or_build(s, true).expect("error");
        s.to_chain_nalgebra()
    }
}

pub trait ToChainNalgebra {
    fn to_chain_nalgebra(&self) -> ChainNalgebra;
}
impl ToChainNalgebra for ResourcesSubDirectory {
    fn to_chain_nalgebra(&self) -> ChainNalgebra {
        ChainNalgebra::new_from_sub_directory(self)
    }
}

pub trait ToChainFromName {
    fn to_chain(&self, robot_name: &str) -> ChainNalgebra;
}
impl ToChainFromName for ResourcesRootDirectory {
    fn to_chain(&self, robot_name: &str) -> ChainNalgebra {
        ChainNalgebra::new_from_root_directory(self, robot_name)
    }
}

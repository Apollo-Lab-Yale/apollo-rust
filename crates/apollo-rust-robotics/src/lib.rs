use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_modules::robot_modules::bevy_modules::first_look_vis_module::ApolloFirstLookVisModule;
use std::path::PathBuf;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_approximations_module::ApolloLinkShapesApproximationsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_distance_statistics_module::ApolloLinkShapesDistanceStatisticsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_max_distance_from_origin_module::ApolloLinkShapesMaxDistanceFromOriginModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_simple_skips_module::ApolloLinkShapesSimpleSkipsModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
pub use apollo_rust_modules::{ResourcesRootDirectory, ResourcesSubDirectory, ResourcesType};
use apollo_rust_preprocessor::PreprocessorModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_distance_statistics_nalgebra_module::ApolloLinkShapesDistanceStatisticsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::ApolloLinkShapesModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_simple_skips_nalgebra_module::ApolloLinkShapesSimpleSkipsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::link_shapes_skips_nalgebra_module::ApolloLinkShapesSkipsNalgebraModule;
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;
pub use apollo_rust_robotics_core::ChainNalgebra;

/// Trait defining builder methods for creating instances of the implementing type from various resources.
pub trait ChainBuildersTrait {
    /// Creates a new instance from the root directory and robot name.
    ///
    /// # Arguments
    /// * `root` - Reference to the `ResourcesRootDirectory`.
    /// * `robot_name` - The name of the robot.
    fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self;

    /// Creates a new instance from a subdirectory.
    ///
    /// # Arguments
    /// * `s` - Reference to the `ResourcesSubDirectory`.
    fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self;

    /// Creates a new instance from a path to a URDD.
    ///
    /// # Arguments
    /// * `path` - The path to the URDD.
    /// * `resources_type` - The type of resources.
    fn new_from_path(path: &PathBuf, resources_type: ResourcesType) -> Self;

    /// Creates a new instance from a path to a URDD with an explicit name.
    ///
    /// # Arguments
    /// * `path` - The path to the URDD.
    /// * `name` - The explicit name of the robot/environment.
    /// * `resources_type` - The type of resources.
    fn new_from_path_with_name(path: &PathBuf, name: &str, resources_type: ResourcesType) -> Self;
}

/// Implementation of `ChainBuildersTrait` for the `ChainNalgebra` type.
impl ChainBuildersTrait for ChainNalgebra {
    fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self {
        let s = root.get_subdirectory(robot_name);
        Self::new_from_sub_directory(&s)
    }

    fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self {
        let mut s = s.clone();
        let apollo_urdf_module = ApolloURDFModule::load_or_build(&s, false).expect("error");
        s.name = apollo_urdf_module.name.clone();

        let urdf_module = ApolloURDFNalgebraModule::from_urdf_module(&apollo_urdf_module);
        let chain_module = ApolloChainModule::load_or_build(&s, false).expect("error");
        let dof_module = ApolloDOFModule::load_or_build(&s, false).expect("error");
        let connections_module = ApolloConnectionsModule::load_or_build(&s, false).expect("error");
        let original_meshes_module =
            ApolloOriginalMeshesModule::load_or_build(&s, false).expect("error");
        let plain_meshes_module = ApolloPlainMeshesModule::load_or_build(&s, false).expect("error");
        let convex_hull_meshes_module =
            ApolloConvexHullMeshesModule::load_or_build(&s, false).expect("error");
        let convex_decomposition_meshes_module =
            ApolloConvexDecompositionMeshesModule::load_or_build(&s, false).expect("error");
        ApolloFirstLookVisModule::load_or_build(&s, false).expect("error");
        let link_shapes_module = ApolloLinkShapesModule::from_mesh_modules(
            &s,
            &convex_hull_meshes_module,
            &convex_decomposition_meshes_module,
        );
        let link_shapes_approximations_module =
            ApolloLinkShapesApproximationsModule::load_or_build(&s, false).expect("error");
        let link_shapes_max_distance_from_origin_module =
            ApolloLinkShapesMaxDistanceFromOriginModule::load_or_build(&s, false).expect("error");
        let link_shapes_distance_statistics_module =
            ApolloLinkShapesDistanceStatisticsModule::load_or_build(&s, false).expect("error");
        let link_shapes_distance_statistics_module = ApolloLinkShapesDistanceStatisticsNalgebraModule::from_link_shapes_distance_statistics_module(&link_shapes_distance_statistics_module);
        let link_shapes_simple_skips_module =
            ApolloLinkShapesSimpleSkipsModule::load_or_build(&s, false).expect("error");
        let link_shapes_simple_skips_nalgebra_module =
            ApolloLinkShapesSimpleSkipsNalgebraModule::from_link_shapes_simple_skips_module(
                &link_shapes_simple_skips_module,
            );
        let link_shapes_skips_module =
            ApolloLinkShapesSkipsModule::load_or_build(&s, false).expect("error");
        let link_shapes_skips_nalgebra_module =
            ApolloLinkShapesSkipsNalgebraModule::from_link_shapes_skips_module(
                &link_shapes_skips_module,
            );
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

    fn new_from_path(path: &PathBuf, resources_type: ResourcesType) -> Self {
        let s = ResourcesSubDirectory::new_from_path(path.clone(), resources_type);
        Self::new_from_sub_directory(&s)
    }

    fn new_from_path_with_name(path: &PathBuf, name: &str, resources_type: ResourcesType) -> Self {
        let s = ResourcesSubDirectory::new_from_path_with_name(
            path.clone(),
            name.to_string(),
            resources_type,
        );
        Self::new_from_sub_directory(&s)
    }
}

/// Trait defining a preprocessing step for refining the link shapes skips module.
pub trait PreprocessForceBuildModulesTrait {
    /// Refines the link shapes skips module and returns the updated instance.
    fn refine_link_shapes_skips_module(self) -> Self;
}

/// Implementation of `PreprocessForceBuildModulesTrait` for the `ChainNalgebra` type.
impl PreprocessForceBuildModulesTrait for ChainNalgebra {
    fn refine_link_shapes_skips_module(self) -> Self {
        let s = &self.resources_sub_directory;
        ApolloLinkShapesSkipsModule::load_or_build(s, true).expect("error");
        s.to_chain_nalgebra()
    }
}

/// Trait defining a conversion from `ResourcesSubDirectory` to `ChainNalgebra`.
pub trait ToChainNalgebra {
    /// Converts the `ResourcesSubDirectory` to a `ChainNalgebra` instance.
    ///
    /// # Returns
    /// A `ChainNalgebra` instance.
    fn to_chain_nalgebra(&self) -> ChainNalgebra;
}

/// Implementation of `ToChainNalgebra` for `ResourcesSubDirectory`.
impl ToChainNalgebra for ResourcesSubDirectory {
    fn to_chain_nalgebra(&self) -> ChainNalgebra {
        ChainNalgebra::new_from_sub_directory(self)
    }
}

/// Trait defining a conversion from `ResourcesRootDirectory` and robot name to `ChainNalgebra`.
pub trait ToChainFromName {
    /// Converts the `ResourcesRootDirectory` and robot name to a `ChainNalgebra` instance.
    ///
    /// # Arguments
    /// * `robot_name` - The name of the robot.
    ///
    /// # Returns
    /// A `ChainNalgebra` instance.
    fn to_chain(&self, robot_name: &str) -> ChainNalgebra;
}

/// Implementation of `ToChainFromName` for `ResourcesRootDirectory`.
impl ToChainFromName for ResourcesRootDirectory {
    fn to_chain(&self, robot_name: &str) -> ChainNalgebra {
        ChainNalgebra::new_from_root_directory(self, robot_name)
    }
}

/// Trait defining a conversion from `PathBuf` to `ChainNalgebra`.
pub trait ToChainFromPath {
    /// Converts the `PathBuf` to a `ChainNalgebra` instance.
    ///
    /// # Arguments
    /// * `resources_type` - The type of resources.
    ///
    /// # Returns
    /// A `ChainNalgebra` instance.
    fn to_chain(&self, resources_type: ResourcesType) -> ChainNalgebra;
}

/// Implementation of `ToChainFromPath` for `PathBuf`.
impl ToChainFromPath for PathBuf {
    fn to_chain(&self, resources_type: ResourcesType) -> ChainNalgebra {
        ChainNalgebra::new_from_path(self, resources_type)
    }
}

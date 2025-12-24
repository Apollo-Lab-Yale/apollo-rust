use ad_trait::AD;
use apollo_rust_modules::robot_modules::bevy_modules::first_look_vis_module::ApolloFirstLookVisModule;
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_modules::robot_modules::urdf_module::ApolloURDFModule;
pub use apollo_rust_modules::{ResourcesRootDirectory, ResourcesSubDirectory, ResourcesType};
use apollo_rust_preprocessor::PreprocessorModule;
use apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait;
use apollo_rust_robotics_core_adtrait::modules_runtime::bounds_adtrait_module::ApolloBoundsModuleADTrait;
use apollo_rust_robotics_core_adtrait::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModuleADTrait;
use std::path::PathBuf;

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

/// Implementation of `ChainBuildersTrait` for the `ChainNalgebraADTrait` type.
impl<A: AD> ChainBuildersTrait for ChainNalgebraADTrait<A> {
    fn new_from_root_directory(root: &ResourcesRootDirectory, robot_name: &str) -> Self {
        let s = root.get_subdirectory(robot_name);
        Self::new_from_sub_directory(&s)
    }

    fn new_from_sub_directory(s: &ResourcesSubDirectory) -> Self {
        let mut s = s.clone();
        let apollo_urdf_module = ApolloURDFModule::load_or_build(&s, false).expect("error");
        s.name = apollo_urdf_module.name.clone();

        let urdf_module =
            ApolloURDFNalgebraModuleADTrait::<A>::from_urdf_module(&apollo_urdf_module);
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
        let bounds_module = ApolloBoundsModule::load_or_build(&s, false).expect("error");
        let bounds_module_ad = ApolloBoundsModuleADTrait::from_apollo_bounds_module(&bounds_module);

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
            bounds_module: bounds_module_ad,
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

pub trait PreprocessForceBuildModulesTrait {
    /// Refines the link shapes skips module and returns the updated instance.
    fn refine_link_shapes_skips_module(self) -> Self;
}

/// Implementation of `PreprocessForceBuildModulesTrait` for the `ChainNalgebra` type.
impl<A: AD> PreprocessForceBuildModulesTrait for ChainNalgebraADTrait<A> {
    fn refine_link_shapes_skips_module(self) -> Self {
        let s = &self.resources_sub_directory;
        ApolloLinkShapesSkipsModule::load_or_build(s, true).expect("error");
        s.to_chain_nalgebra_adtrait()
    }
}

/// Trait defining a conversion from `ResourcesSubDirectory` to `ChainNalgebra`.
pub trait ToChainNalgebraADTrait {
    /// Converts the `ResourcesSubDirectory` to a `ChainNalgebra` instance.
    ///
    /// # Returns
    /// A `ChainNalgebra` instance.
    fn to_chain_nalgebra_adtrait<A: AD>(&self) -> ChainNalgebraADTrait<A>;
}

/// Implementation of `ToChainNalgebra` for `ResourcesSubDirectory`.
impl ToChainNalgebraADTrait for ResourcesSubDirectory {
    fn to_chain_nalgebra_adtrait<A: AD>(&self) -> ChainNalgebraADTrait<A> {
        ChainNalgebraADTrait::new_from_sub_directory(self)
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
    fn to_chain<A: AD>(&self, robot_name: &str) -> ChainNalgebraADTrait<A>;
}

/// Implementation of `ToChainFromName` for `ResourcesRootDirectory`.
impl ToChainFromName for ResourcesRootDirectory {
    fn to_chain<A: AD>(&self, robot_name: &str) -> ChainNalgebraADTrait<A> {
        ChainNalgebraADTrait::new_from_root_directory(self, robot_name)
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
    fn to_chain<A: AD>(&self, resources_type: ResourcesType) -> ChainNalgebraADTrait<A>;
}

/// Implementation of `ToChainFromPath` for `PathBuf`.
impl ToChainFromPath for PathBuf {
    fn to_chain<A: AD>(&self, resources_type: ResourcesType) -> ChainNalgebraADTrait<A> {
        ChainNalgebraADTrait::new_from_path(self, resources_type)
    }
}

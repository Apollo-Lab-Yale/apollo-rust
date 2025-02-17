use std::sync::Arc;
use ad_trait::AD;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use crate::modules_runtime::bounds_adtrait_module::ApolloBoundsADTraitModule;
use crate::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;

pub mod modules_runtime;
pub mod robot_functions;
pub mod modules;

#[derive(Clone)]
pub struct ChainNalgebraADTrait<A: AD> {
    pub resources_sub_directory: ResourcesSubDirectory,
    pub urdf_module: ApolloURDFNalgebraModule<A>,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub connections_module: ApolloConnectionsModule,
    pub original_meshes_module: ApolloOriginalMeshesModule,
    pub plain_meshes_module: ApolloPlainMeshesModule,
    pub convex_hull_meshes_module: ApolloConvexHullMeshesModule,
    pub convex_decomposition_meshes_module: ApolloConvexDecompositionMeshesModule,
    pub bounds_module: ApolloBoundsADTraitModule<A>
}
impl<A: AD> ChainNalgebraADTrait<A> {
    pub fn to_arc_chain(self) -> Arc<ChainNalgebraADTrait<A>> {
        Arc::new(self)
    }
}
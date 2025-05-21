use std::sync::Arc;
use ad_trait::AD;
use apollo_rust_linalg_adtrait::V;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::chain_module::ApolloChainModule;
use apollo_rust_modules::robot_modules::connections_module::ApolloConnectionsModule;
use apollo_rust_modules::robot_modules::dof_module::ApolloDOFModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_decomposition_meshes_module::ApolloConvexDecompositionMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::original_meshes_module::ApolloOriginalMeshesModule;
use apollo_rust_modules::robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::ISE3q;
use crate::modules_runtime::bounds_adtrait_module::ApolloBoundsModuleADTrait;
use crate::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModuleADTrait;
use crate::robot_functions::robot_kinematics_functions::RobotKinematicsFunctions;

pub mod modules_runtime;
pub mod robot_functions;
pub mod modules;

#[derive(Clone)]
pub struct ChainNalgebraADTrait<A: AD> {
    pub resources_sub_directory: ResourcesSubDirectory,
    pub urdf_module: ApolloURDFNalgebraModuleADTrait<A>,
    pub chain_module: ApolloChainModule,
    pub dof_module: ApolloDOFModule,
    pub connections_module: ApolloConnectionsModule,
    pub original_meshes_module: ApolloOriginalMeshesModule,
    pub plain_meshes_module: ApolloPlainMeshesModule,
    pub convex_hull_meshes_module: ApolloConvexHullMeshesModule,
    pub convex_decomposition_meshes_module: ApolloConvexDecompositionMeshesModule,
    pub bounds_module: ApolloBoundsModuleADTrait<A>
}
impl<A: AD> ChainNalgebraADTrait<A> {
    pub fn to_arc_chain(self) -> Arc<ChainNalgebraADTrait<A>> {
        Arc::new(self)
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> ChainNalgebraADTrait<A2> {
        ChainNalgebraADTrait {
            resources_sub_directory: self.resources_sub_directory.clone(),
            urdf_module: self.urdf_module.to_other_ad_type::<A2>(),
            chain_module: self.chain_module.clone(),
            dof_module: self.dof_module.clone(),
            connections_module: self.connections_module.clone(),
            original_meshes_module: self.original_meshes_module.clone(),
            plain_meshes_module: self.plain_meshes_module.clone(),
            convex_hull_meshes_module: self.convex_hull_meshes_module.clone(),
            convex_decomposition_meshes_module: self.convex_decomposition_meshes_module.clone(),
            bounds_module: self.bounds_module.to_other_ad_type::<A2>(),
        }
    }
}
impl<A: AD> ChainNalgebraADTrait<A> {
    #[inline(always)]
    pub fn num_dofs(&self) -> usize {
        self.dof_module.num_dofs
    }

    #[inline(always)]
    pub fn zeros_state(&self) -> V<A> {
        V::from_column_slice(&vec![A::zero(); self.num_dofs()])
    }

    #[inline(always)]
    pub fn fk(&self, state: &V<A>) -> Vec<ISE3q<A>> {
        RobotKinematicsFunctions::fk(state, &self.urdf_module, &self.chain_module, &self.dof_module)
    }

    #[inline(always)]
    pub fn reverse_of_fk(&self, link_frame: &Vec<ISE3q<A>>) -> V<A> {
        RobotKinematicsFunctions::reverse_of_fk(link_frame, &self.urdf_module, &self.chain_module, &self.dof_module)
    }
}
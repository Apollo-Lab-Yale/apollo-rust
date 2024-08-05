use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::mesh_modules::convex_hull_meshes_module::ApolloConvexHullMeshesModule;


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentConvexHullMeshesModule(pub ApolloConvexHullMeshesModule);
use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::mesh_modules::plain_meshes_module::ApolloPlainMeshesModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentPlainMeshesModule(pub ApolloPlainMeshesModule);
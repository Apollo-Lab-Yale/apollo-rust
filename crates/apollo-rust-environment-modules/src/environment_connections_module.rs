use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::connections_module::ApolloConnectionsModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentConnectionsModule(pub ApolloConnectionsModule);
use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::dof_module::ApolloDOFModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentDOFModule(pub ApolloDOFModule);
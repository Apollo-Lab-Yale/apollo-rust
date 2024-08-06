use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::bounds_module::ApolloBoundsModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentBoundsModule(pub ApolloBoundsModule);
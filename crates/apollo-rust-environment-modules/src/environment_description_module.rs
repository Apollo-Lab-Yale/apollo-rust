use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentDescriptionModule(pub ApolloURDFModule);


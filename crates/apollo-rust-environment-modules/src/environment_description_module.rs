/*
use serde::{Deserialize, Serialize};
use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentDescriptionModule {
    pub urdf_module: ApolloURDFModule,
    pub link_scales: Vec<[f64; 3]>,
    pub link_simulation_modes: Vec<EnvironmentLinkSimulationMode>
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentLinkSimulationMode {
    Active,
    Passive
}
*/
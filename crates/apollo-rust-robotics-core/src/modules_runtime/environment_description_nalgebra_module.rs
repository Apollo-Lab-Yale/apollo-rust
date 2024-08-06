use serde::{Deserialize, Serialize};
use apollo_rust_environment_modules::environment_description_module::{ApolloEnvironmentDescriptionModule, EnvironmentLinkSimulationMode};
use crate::modules_runtime::urdf_nalgebra_module::ApolloURDFNalgebraModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentDescriptionNalgebraModule {
    pub urdf_module: ApolloURDFNalgebraModule,
    pub link_scales: Vec<[f64; 3]>,
    pub link_simulation_modes: Vec<EnvironmentLinkSimulationMode>
}
impl ApolloEnvironmentDescriptionNalgebraModule {
    pub fn from_environment_description_module(environment_description_module: &ApolloEnvironmentDescriptionModule) -> Self {
        let urdf_module = ApolloURDFNalgebraModule::from_urdf_module(&environment_description_module.urdf_module);
        Self {
            urdf_module,
            link_scales: environment_description_module.link_scales.clone(),
            link_simulation_modes: environment_description_module.link_simulation_modes.clone(),
        }
    }
}
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentLinkSimulationModesModule {
    pub modes: Vec<Option<EnvironmentLinkSimulationModes>>
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentLinkSimulationModes {
    Active,
    Passive
}
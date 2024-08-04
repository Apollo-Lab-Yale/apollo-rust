use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloEnvironmentLinkSimulationModesModule {
    pub modes: Vec<Option<EnvironmentLinkSimulationMode>>
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum EnvironmentLinkSimulationMode {
    Active,
    Passive
}
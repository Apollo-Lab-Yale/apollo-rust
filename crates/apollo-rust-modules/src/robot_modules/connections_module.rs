use serde::{Deserialize, Serialize};


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloConnectionsModule {
    pub link_connection_paths: Vec<Vec<Option<Vec<usize>>>>
}
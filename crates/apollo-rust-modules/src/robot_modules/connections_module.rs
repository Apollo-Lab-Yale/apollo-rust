use serde::{Deserialize, Serialize};

/// Struct representing a module for managing connection paths between links in a system.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloConnectionsModule {
    /// A 2D vector where each element is an `Option` containing a vector of indices representing the connection path between two links.
    ///
    /// - The outer vector corresponds to a set of links.
    /// - Each inner vector corresponds to a connection path between two links.
    pub link_connection_paths: Vec<Vec<Option<Vec<usize>>>>,
}

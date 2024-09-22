use serde::{Deserialize, Serialize};

/// Struct representing a Degrees of Freedom (DOF) module, managing the mapping between DOFs and joints.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloDOFModule {
    /// The total number of degrees of freedom (DOFs) in the system.
    pub num_dofs: usize,

    /// A vector mapping each DOF index to its corresponding joint index.
    pub dof_idx_to_joint_idx_mapping: Vec<usize>,

    /// A vector where each element is a vector of DOF indices associated with a particular joint index.
    pub joint_idx_to_dof_idxs_mapping: Vec<Vec<usize>>,
}

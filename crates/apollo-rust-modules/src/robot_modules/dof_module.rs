use serde::{Deserialize, Serialize};


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloDOFModule {
    pub num_dofs: usize,
    pub dof_idx_to_joint_idx_mapping: Vec<usize>,
    pub joint_idx_to_dof_idxs_mapping: Vec<Vec<usize>>
}


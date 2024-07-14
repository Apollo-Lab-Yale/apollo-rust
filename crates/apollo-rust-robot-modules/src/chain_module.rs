use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloChainModule {
    pub links_in_chain: Vec<ApolloLinkInChain>,
    pub joints_in_chain: Vec<ApolloJointInChain>,
    pub kinematic_hierarchy: Vec<Vec<usize>>,
    pub root_idx: usize
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloLinkInChain {
    pub name: String,
    pub link_idx: usize,
    pub parent_joint_idx: Option<usize>,
    pub parent_link_idx: Option<usize>,
    pub children_joint_idxs: Vec<usize>,
    pub children_link_idxs: Vec<usize>
}
impl ApolloLinkInChain {
    #[inline(always)]
    pub fn name(&self) -> &str {
        &self.name
    }

    #[inline(always)]
    pub fn link_idx(&self) -> usize {
        self.link_idx
    }

    #[inline(always)]
    pub fn parent_joint_idx(&self) -> Option<usize> {
        self.parent_joint_idx
    }

    #[inline(always)]
    pub fn parent_link_idx(&self) -> Option<usize> {
        self.parent_link_idx
    }

    #[inline(always)]
    pub fn children_joint_idxs(&self) -> &Vec<usize> {
        &self.children_joint_idxs
    }

    #[inline(always)]
    pub fn children_link_idxs(&self) -> &Vec<usize> {
        &self.children_link_idxs
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloJointInChain {
    pub joint_name: String,
    pub joint_idx: usize,
    pub parent_link_name: String,
    pub parent_link_idx: usize,
    pub child_link_name: String,
    pub child_link_idx: usize,
}
impl ApolloJointInChain {
    #[inline(always)]
    pub fn joint_name(&self) -> &str {
        &self.joint_name
    }

    #[inline(always)]
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
    }

    #[inline(always)]
    pub fn parent_link_name(&self) -> &str {
        &self.parent_link_name
    }

    #[inline(always)]
    pub fn parent_link_idx(&self) -> usize {
        self.parent_link_idx
    }

    #[inline(always)]
    pub fn child_link_name(&self) -> &str {
        &self.child_link_name
    }

    #[inline(always)]
    pub fn child_link_idx(&self) -> usize {
        self.child_link_idx
    }
}
use serde::{Deserialize, Serialize};

/// Struct representing a kinematic chain module, which contains links and joints and defines the hierarchical structure of the chain.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloChainModule {
    /// A vector of links in the chain.
    pub links_in_chain: Vec<ApolloLinkInChain>,

    /// A vector of joints in the chain.
    pub joints_in_chain: Vec<ApolloJointInChain>,

    /// A vector of vectors representing the kinematic hierarchy. Each sub-vector contains indices pointing to hierarchical connections.
    pub kinematic_hierarchy: Vec<Vec<usize>>,

    /// Index of the root link in the chain.
    pub root_idx: usize,
}

/// Struct representing a link in a kinematic chain, which may have parent and child joints and links.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloLinkInChain {
    /// Name of the link.
    pub name: String,

    /// Index of the link in the chain.
    pub link_idx: usize,

    /// Index of the parent joint, if any.
    pub parent_joint_idx: Option<usize>,

    /// Index of the parent link, if any.
    pub parent_link_idx: Option<usize>,

    /// A vector of indices of the child joints.
    pub children_joint_idxs: Vec<usize>,

    /// A vector of indices of the child links.
    pub children_link_idxs: Vec<usize>,
}

impl ApolloLinkInChain {
    /// Returns the name of the link.
    #[inline(always)]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Returns the index of the link.
    #[inline(always)]
    pub fn link_idx(&self) -> usize {
        self.link_idx
    }

    /// Returns the index of the parent joint, if any.
    #[inline(always)]
    pub fn parent_joint_idx(&self) -> Option<usize> {
        self.parent_joint_idx
    }

    /// Returns the index of the parent link, if any.
    #[inline(always)]
    pub fn parent_link_idx(&self) -> Option<usize> {
        self.parent_link_idx
    }

    /// Returns a reference to the vector of indices of child joints.
    #[inline(always)]
    pub fn children_joint_idxs(&self) -> &Vec<usize> {
        &self.children_joint_idxs
    }

    /// Returns a reference to the vector of indices of child links.
    #[inline(always)]
    pub fn children_link_idxs(&self) -> &Vec<usize> {
        &self.children_link_idxs
    }
}

/// Struct representing a joint in a kinematic chain, which connects a parent and a child link.
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ApolloJointInChain {
    /// Name of the joint.
    pub joint_name: String,

    /// Index of the joint in the chain.
    pub joint_idx: usize,

    /// Name of the parent link connected by this joint.
    pub parent_link_name: String,

    /// Index of the parent link connected by this joint.
    pub parent_link_idx: usize,

    /// Name of the child link connected by this joint.
    pub child_link_name: String,

    /// Index of the child link connected by this joint.
    pub child_link_idx: usize,
}

impl ApolloJointInChain {
    /// Returns the name of the joint.
    #[inline(always)]
    pub fn joint_name(&self) -> &str {
        &self.joint_name
    }

    /// Returns the index of the joint.
    #[inline(always)]
    pub fn joint_idx(&self) -> usize {
        self.joint_idx
    }

    /// Returns the name of the parent link connected by this joint.
    #[inline(always)]
    pub fn parent_link_name(&self) -> &str {
        &self.parent_link_name
    }

    /// Returns the index of the parent link connected by this joint.
    #[inline(always)]
    pub fn parent_link_idx(&self) -> usize {
        self.parent_link_idx
    }

    /// Returns the name of the child link connected by this joint.
    #[inline(always)]
    pub fn child_link_name(&self) -> &str {
        &self.child_link_name
    }

    /// Returns the index of the child link connected by this joint.
    #[inline(always)]
    pub fn child_link_idx(&self) -> usize {
        self.child_link_idx
    }
}

use std::collections::HashMap;
use std::hash::{Hash, Hasher};
use bevy::ecs::query::{QueryFilter};
use bevy::prelude::{Component, Entity, Query};
use apollo_rust_algs::power_set;
use crate::apollo_bevy_utils::chain::ChainMeshesRepresentation;
use crate::apollo_bevy_utils::meshes::MeshType;

#[derive(Component)]
pub struct Signatures(pub HashMap<Signature, ()>);

/*
#[derive(Clone, Debug, Component, PartialEq, Eq, Hash)]
pub enum Signature {
    ChainLinkMesh,
    ChainLinkMeshInstance { chain_instance_idx: usize },
    ChainLinkMeshInstanceAndLink { chain_instance_idx: usize, link_idx: usize },
    ChainLinkPlainMesh,
    ChainLinkPlainMeshInstance { chain_instance_idx: usize },
    ChainLinkPlainMeshLink { chain_instance_idx: usize, link_idx: usize },
    ChainLinkConvexHullMesh,
    ChainLinkConvexHullMeshInstance { chain_instance_idx: usize },
    ChainLinkConvexHullMeshLink { chain_instance_idx: usize, link_idx: usize },
    ChainLinkConvexDecompositionMesh,
    ChainLinkConvexDecompositionMeshInstance { chain_instance_idx: usize },
    ChainLinkConvexDecompositionMeshLink { chain_instance_idx: usize, link_idx: usize },
    ChainLinkConvexDecompositionMeshSubcomponent { chain_instance_idx: usize, link_idx: usize, subcomponent_idx: usize },
    Test { mesh_types: Vec<MeshType> }
}
*/

#[derive(Debug, Clone, PartialOrd, Ord, PartialEq, Eq, Hash)]
pub enum Signature {
    ChainLinkMesh { components: ChainMeshComponents }
}
impl Signature {
    pub fn new_chain_link_mesh(c: Vec<ChainMeshComponent>) -> Self {
        Self::ChainLinkMesh { components: ChainMeshComponents::new(c) }
    }
}

#[derive(Debug, Clone, Eq, PartialOrd, Ord)]
pub struct ChainMeshComponents(pub Vec<ChainMeshComponent>);
impl ChainMeshComponents {
    pub fn new(field0: Vec<ChainMeshComponent>) -> Self {
        Self(field0)
    }
    pub fn get_power_set_general(original_set: Vec<ChainMeshComponent>) -> Vec<Self> {
        let p = power_set(original_set);
        p.iter().map(|x| ChainMeshComponents(x.clone())).collect()
    }
    pub fn get_power_set_default(chain_meshes_representation: ChainMeshesRepresentation, mesh_type: MeshType, chain_instance_idx: usize, link_idx: usize, subcomponent_idx: usize) -> Vec<Self> {
        let v = vec![
            ChainMeshComponent::ChainMeshesRepresentation(chain_meshes_representation.clone()),
            ChainMeshComponent::MeshType(mesh_type.clone()),
            ChainMeshComponent::LinkIdx(link_idx),
            ChainMeshComponent::SubcomponentIdx(subcomponent_idx.clone()),
            ChainMeshComponent::ChainInstanceIdx(chain_instance_idx.clone())
        ];

        let p = power_set(v);
        p.iter().map(|x| ChainMeshComponents(x.clone())).collect()
    }
}
impl PartialEq for ChainMeshComponents {
    fn eq(&self, other: &Self) -> bool {
        if self.0.len() != other.0.len() { return false; }
        let mut self_clone = self.0.clone();
        let mut other_clone = other.0.clone();
        self_clone.sort();
        other_clone.sort();

        return self_clone == other_clone;
    }
}
impl Hash for ChainMeshComponents {
    fn hash<H: Hasher>(&self, state: &mut H) {
        let mut components_clone = self.0.clone();
        components_clone.sort();
        components_clone.hash(state)
    }
}

#[derive(Clone, Debug, Component, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ChainMeshComponent {
    ChainInstanceIdx(usize),
    LinkIdx(usize),
    SubcomponentIdx(usize),
    MeshType(MeshType),
    ChainMeshesRepresentation(ChainMeshesRepresentation)
}

pub trait SignatureQueryTrait {
    fn get_all_entities_with_signature(&self, signature: Signature) -> Vec<Entity>;
}
impl<F: QueryFilter> SignatureQueryTrait for Query<'_, '_, (&Signatures, Entity), F> {
    fn get_all_entities_with_signature(&self, signature: Signature) -> Vec<Entity> {
        let mut out = vec![];

        self.iter().for_each(|x| {
            // if x.0.0.contains(&signature) { out.push(x.1); }
            if x.0.0.get(&signature).is_some() { out.push(x.1); }
        });

        out
    }
}
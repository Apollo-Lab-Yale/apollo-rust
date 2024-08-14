use std::collections::HashMap;
use bevy::ecs::query::{QueryFilter};
use bevy::prelude::{Component, Entity, Query};

#[derive(Component)]
pub struct Signatures(pub HashMap<Signature, ()>);

#[derive(Clone, Debug, Component, PartialEq, Eq, Hash, Copy)]
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
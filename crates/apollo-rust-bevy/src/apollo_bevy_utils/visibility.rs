use bevy::prelude::{Component, Entity, Query, Res, ResMut, Resource, Visibility};
use crate::apollo_bevy_utils::signatures::{Signature, SignatureQueryTrait, Signatures};

#[derive(Clone, Debug, Resource)]
pub struct VisibilityEngine {
    momentary_requests: Vec<VisibilityRequest>,
    base_change_requests: Vec<VisibilityRequest>
}
impl VisibilityEngine {
    pub fn new() -> Self {
        Self {
            momentary_requests: vec![],
            base_change_requests: vec![],
        }
    }

    pub fn add_momentary_request(&mut self, request: VisibilityRequest) {
        let idx = self.momentary_requests.iter().position(|x| x.signature == request.signature);
        match idx {
            None => {
                self.momentary_requests.push(request);
            }
            Some(idx) => {
                let curr_request = &self.momentary_requests[idx];
                if &request.visibility_request_type > &curr_request.visibility_request_type {
                    self.momentary_requests[idx] = request;
                }
            }
        }
    }

    pub fn add_base_change_request(&mut self, request: VisibilityRequest) {
        let idx = self.base_change_requests.iter().position(|x| x.signature == request.signature);
        match idx {
            None => {
                self.base_change_requests.push(request);
            }
            Some(idx) => {
                let curr_request = &self.base_change_requests[idx];
                if &request.visibility_request_type > &curr_request.visibility_request_type {
                    self.base_change_requests[idx] = request;
                }
            }
        }
    }

    pub fn clear(&mut self) {
        self.momentary_requests.clear();
        self.base_change_requests.clear();
    }
}

#[derive(Clone, Debug)]
pub struct VisibilityRequest {
    pub visibility_request_type: VisibilityMode,
    pub signature: Signature
}
impl VisibilityRequest {
    pub fn new(visibility_request_type: VisibilityMode, signature: Signature) -> Self {
        Self { visibility_request_type, signature }
    }
}

#[derive(Component, Clone, Debug)]
pub enum BaseVisibility {
    On,
    Off
}

#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum VisibilityMode {
    Toggle,
    On,
    Off
}

pub struct VisibilitySystems;
impl VisibilitySystems {
    pub fn system_set_momentary_visibility_request_changes(engine: Res<VisibilityEngine>, mut query_visibility: Query<&mut Visibility>, query_signatures: Query<(&Signatures, Entity)>) {
        for momentary_request in &engine.momentary_requests {
            let e = query_signatures.get_all_entities_with_signature(momentary_request.signature);
            for ee in e {
                let mut x = query_visibility.get_mut(ee).expect("error");
                match &momentary_request.visibility_request_type {
                    VisibilityMode::On => { *x = Visibility::Visible }
                    VisibilityMode::Off => { *x = Visibility::Hidden }
                    VisibilityMode::Toggle => {
                        if *x == Visibility::Visible {
                            *x = Visibility::Hidden
                        } else if *x == Visibility::Hidden {
                            *x = Visibility::Visible
                        }
                    }
                }
            }
        }
    }

    pub fn system_set_base_visibility_request_changes(engine: Res<VisibilityEngine>, mut query_base_visibility: Query<&mut BaseVisibility>, query_signatures: Query<(&Signatures, Entity)>) {
        for request in &engine.base_change_requests {
            let e = query_signatures.get_all_entities_with_signature(request.signature);
            for ee in e {
                let mut x = query_base_visibility.get_mut(ee).expect("error");
                match &request.visibility_request_type {
                    VisibilityMode::On => { *x = BaseVisibility::On }
                    VisibilityMode::Off => { *x = BaseVisibility::Off }
                    VisibilityMode::Toggle => {
                        match &*x {
                            BaseVisibility::On => { *x = BaseVisibility::Off }
                            BaseVisibility::Off => { *x = BaseVisibility::On }
                        }
                    }
                }
            }
        }
    }

    pub fn system_clear_visibility_request_changes(mut engine: ResMut<VisibilityEngine>) {
        engine.clear();
    }

    pub fn system_reset_base_visibilities(query_base_visibility: Query<(&BaseVisibility, Entity)>, mut query_visibility: Query<&mut Visibility>) {
        query_base_visibility.iter().for_each(|(v, e)| {
            if let Ok(mut ee) = query_visibility.get_mut(e) {
                match v {
                    BaseVisibility::On => { *ee = Visibility::Visible }
                    BaseVisibility::Off => { *ee = Visibility::Hidden }
                }
            }
        });
    }
}
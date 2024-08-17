use bevy::pbr::StandardMaterial;
use bevy::prelude::{Alpha, AlphaMode, Assets, Color, Component, Entity, Handle, Query, Res, ResMut, Resource};
use crate::apollo_bevy_utils::signatures::{Signature, SignatureQueryTrait, Signatures};

#[derive(Clone, Debug, Resource)]
pub struct ColorChangeEngine {
    momentary_requests: Vec<ColorChangeRequest>,
    base_change_requests: Vec<ColorChangeRequest>
}
impl ColorChangeEngine {
    pub fn new() -> Self {
        Self {
            momentary_requests: vec![],
            base_change_requests: vec![],
        }
    }

    pub fn add_momentary_request(&mut self, request: ColorChangeRequest) {
        let idx = self.momentary_requests.iter().position(|x| x.signature == request.signature);
        match idx {
            None => {
                self.momentary_requests.push(request);
            }
            Some(idx) => {
                let curr_request = &self.momentary_requests[idx];
                if &request.color_change_request_type > &curr_request.color_change_request_type {
                    self.momentary_requests[idx] = request;
                }
            }
        }
    }

    pub fn add_base_change_request(&mut self, request: ColorChangeRequest) {
        let idx = self.base_change_requests.iter().position(|x| x.signature == request.signature);
        match idx {
            None => {
                self.base_change_requests.push(request);
            }
            Some(idx) => {
                let curr_request = &self.base_change_requests[idx];
                if &request.color_change_request_type > &curr_request.color_change_request_type {
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
pub struct ColorChangeRequest {
    pub color_change_request_type: ColorChangeRequestType,
    pub signature: Signature
}
impl ColorChangeRequest {
    pub fn new(color_change_request_type: ColorChangeRequestType, signature: Signature) -> Self {
        Self { color_change_request_type, signature }
    }
}

#[derive(Component, Clone, Debug)]
pub struct BaseColor(pub Color);

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum ColorChangeRequestType {
    LowPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    LowPriorityAlpha(f32),
    MediumPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    MediumPriorityAlpha(f32),
    HighPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    HighPriorityAlpha(f32),
}
impl ColorChangeRequestType {
    pub fn low_priority_color(r: f32, g: f32, b: f32, a: f32) -> Self {
        ColorChangeRequestType::LowPriorityColor { r, g, b, a }
    }

    pub fn low_priority_alpha(alpha: f32) -> Self {
        ColorChangeRequestType::LowPriorityAlpha(alpha)
    }

    pub fn medium_priority_color(r: f32, g: f32, b: f32, a: f32) -> Self {
        ColorChangeRequestType::MediumPriorityColor { r, g, b, a }
    }

    pub fn medium_priority_alpha(alpha: f32) -> Self {
        ColorChangeRequestType::MediumPriorityAlpha(alpha)
    }

    pub fn high_priority_color(r: f32, g: f32, b: f32, a: f32) -> Self {
        ColorChangeRequestType::HighPriorityColor { r, g, b, a }
    }

    pub fn high_priority_alpha(alpha: f32) -> Self {
        ColorChangeRequestType::HighPriorityAlpha(alpha)
    }
}

pub struct ColorChangeSystems;
impl ColorChangeSystems {
    pub fn system_set_momentary_color_request_changes(engine: Res<ColorChangeEngine>, mut materials: ResMut<Assets<StandardMaterial>>, query_materials: Query<&Handle<StandardMaterial>>, query_signatures: Query<(&Signatures, Entity)>) {
        let mut requests = engine.momentary_requests.clone();
        requests.sort_by(|x, y| x.color_change_request_type.partial_cmp(&y.color_change_request_type).unwrap() );
        for momentary_request in &requests {
            let e = query_signatures.get_all_entities_with_signature(momentary_request.signature.clone());
            for ee in e {
                let x = query_materials.get(ee);
                if let Ok(x) = x {
                    let m = materials.get_mut(x).expect("error");
                    match &momentary_request.color_change_request_type {
                        ColorChangeRequestType::LowPriorityColor { r, g, b, a } => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color = Color::srgba(*r, *g, *b, *a);
                        }
                        ColorChangeRequestType::LowPriorityAlpha(a) => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color.set_alpha(*a);
                        }
                        ColorChangeRequestType::MediumPriorityColor { r, g, b, a } => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color = Color::srgba(*r, *g, *b, *a);
                        }
                        ColorChangeRequestType::MediumPriorityAlpha(a) => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color.set_alpha(*a);
                        }
                        ColorChangeRequestType::HighPriorityColor { r, g, b, a } => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color = Color::srgba(*r, *g, *b, *a);
                        }
                        ColorChangeRequestType::HighPriorityAlpha(a) => {
                            if *a < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                            m.base_color.set_alpha(*a);
                        }
                    }
                }
            }
        }
    }

    pub fn system_set_base_color_request_changes(engine: Res<ColorChangeEngine>, mut query_base_colors: Query<&mut BaseColor>, query_signatures: Query<(&Signatures, Entity)>) {
        let mut requests = engine.base_change_requests.clone();
        requests.sort_by(|x, y| x.color_change_request_type.partial_cmp(&y.color_change_request_type).unwrap() );
        for request in &requests {
            let e = query_signatures.get_all_entities_with_signature(request.signature.clone());
            for ee in e {
                let mut x = query_base_colors.get_mut(ee).expect("error");
                match &request.color_change_request_type {
                    ColorChangeRequestType::LowPriorityColor { r, g, b, a } => {
                        x.0 = Color::srgba(*r, *g, *b, *a);
                    }
                    ColorChangeRequestType::LowPriorityAlpha(a) => {
                        x.0.set_alpha(*a);
                    }
                    ColorChangeRequestType::MediumPriorityColor { r, g, b, a } => {
                        x.0 = Color::srgba(*r, *g, *b, *a);
                    }
                    ColorChangeRequestType::MediumPriorityAlpha(a) => {
                        x.0.set_alpha(*a);
                    }
                    ColorChangeRequestType::HighPriorityColor { r, g, b, a } => {
                        x.0 = Color::srgba(*r, *g, *b, *a);
                    }
                    ColorChangeRequestType::HighPriorityAlpha(a) => {
                        x.0.set_alpha(*a);
                    }
                }
            }
        }
    }

    pub fn system_clear_color_request_changes(mut engine: ResMut<ColorChangeEngine>) {
        engine.clear();
    }

    pub fn system_reset_base_colors(query_base_color: Query<(&BaseColor, Entity)>, mut materials: ResMut<Assets<StandardMaterial>>, query_materials: Query<&Handle<StandardMaterial>>) {
        query_base_color.iter().for_each(|(v, e)| {
            if let Ok(ee) = query_materials.get(e) {
                let m = materials.get_mut(ee);
                if let Some(m) = m {
                    if v.0.alpha() < 1.0 { m.alpha_mode = AlphaMode::Blend; }
                    else { m.alpha_mode = AlphaMode::Opaque; }
                    m.base_color = v.0.clone();
                }
            }
        });
    }
}



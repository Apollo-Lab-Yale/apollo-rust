use bevy::prelude::{Color, Component};

pub struct ColorEngine {

}


#[derive(Component, Clone, Debug)]
pub struct BaseColor(pub Color);

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub enum ColorRequestType {
    LowPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    LowPriorityAlpha(f32),
    MediumPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    MediumPriorityAlpha(f32),
    HighPriorityColor { r: f32, g: f32, b: f32, a: f32 },
    HighPriorityAlpha(f32),
}

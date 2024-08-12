use bevy::prelude::{Query, ResMut, Resource, Window, With};
use bevy::window::PrimaryWindow;
use bevy_egui::egui::{Ui};

#[derive(Resource)]
pub struct CursorIsOverEgui(pub bool);

pub fn set_cursor_is_over_egui_default(ui: &Ui,
                                       cursor_is_over_egui: &mut ResMut<CursorIsOverEgui>,
                                       window_query: &Query<&Window, With<PrimaryWindow>>) {
    set_cursor_is_over_egui_general(ui, 20.0, 20.0, 40.0, 20.0, cursor_is_over_egui, window_query)
}

pub fn set_cursor_is_over_egui_general(ui: &Ui,
                                       right_buffer: f32,
                                       left_buffer: f32,
                                       top_buffer: f32,
                                       bottom_buffer: f32,
                                       cursor_is_over_egui: &mut ResMut<CursorIsOverEgui>,
                                       window_query: &Query<&Window, With<PrimaryWindow>>) {
    let Ok(window) = window_query.get_single() else { return; };

    let rect = ui.min_rect();
    let cursor = window.cursor_position();
    match cursor {
        None => {  }
        Some(cursor) => {
            let x = cursor.x;
            let y = cursor.y;

            let right = rect.right();
            let left = rect.left();
            let top = rect.top();
            let bottom = rect.bottom();

            if x < right + right_buffer && x > left - left_buffer && y < bottom + bottom_buffer && y > top - top_buffer {
                cursor_is_over_egui.0 = true;
            }
        }
    }
}

pub fn reset_cursor_is_over_egui(mut cursor_is_over_egui: ResMut<CursorIsOverEgui>) { cursor_is_over_egui.0 = false; }
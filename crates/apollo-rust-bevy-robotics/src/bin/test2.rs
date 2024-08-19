use std::path::PathBuf;
use std::sync::Arc;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::prelude::{App, AppExit, ButtonInput, EventWriter, KeyCode, Res, ResMut, Update};
use apollo_rust_bevy::{ApolloBevyTrait, get_default_mesh_specs};
use apollo_rust_bevy::apollo_bevy_utils::chain::{BevyChainLinkVisibilitySelector, BevyChainProximityVisualizer, BevyChainSlidersEgui, ChainMeshesRepresentation};
use apollo_rust_bevy::apollo_bevy_utils::colors::{ColorChangeEngine};
use apollo_rust_bevy::apollo_bevy_utils::signatures::{ChainMeshComponent, ChainMeshComponents, Signature};
use apollo_rust_bevy::apollo_bevy_utils::visibility::{VisibilityChangeEngine, VisibilityChangeRequest, VisibilityChangeRequestType};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let r = ResourcesRootDirectory::new(PathBuf::new_from_default_apollo_robots_dir());
    let s = r.get_subdirectory("b1");
    let chain = s.to_chain_nalgebra();
    let chain_arc = Arc::new(chain.clone());

    let mut app = App::new()
        .apollo_bevy_robotics_base(false);

    app = app.apollo_bevy_spawn_robot(&chain, 0, ISE3q::new(I3::from_slices_euler_angles(&[0.,0.,0.], &[0.,0.,0.])), get_default_mesh_specs(), &PathBuf::new_from_default_apollo_bevy_assets_dir());
    // app = app.apollo_bevy_spawn_robot(&chain, 1, ISE3q::new(I3::from_slices_euler_angles(&[0.,1.,0.], &[-1.,0.,0.])), get_default_mesh_specs(), &PathBuf::new_from_default_apollo_bevy_assets_dir());

    app.add_systems(Update, BevyChainSlidersEgui {
        chain_instance_idx: 0,
        chain: chain_arc.clone(),
    }.get_system_side_panel_left(|_, _| { }));

    // app.add_systems(Update, BevyChainLinkVisibilitySelector {
    //     chain_instance_idx: 0,
    //     chain: chain_arc.clone(),
    // }.get_system_side_panel_left());

    app.add_systems(Update, BevyChainProximityVisualizer {
        chain_instance_idx_a: 0,
        chain_a: chain_arc.clone(),
        chain_instance_idx_b: 0,
        chain_b: chain_arc.clone(),
    }.get_system_side_panel_left());

    app.add_plugins(FrameTimeDiagnosticsPlugin::default());
    app.add_plugins(LogDiagnosticsPlugin::default());

    app.add_systems(Update, |keys: Res<ButtonInput<KeyCode>>, mut engine: ResMut<VisibilityChangeEngine>, _color: ResMut<ColorChangeEngine>, mut exit: EventWriter<AppExit>| {
        if keys.just_pressed(KeyCode::KeyM) {
            // engine.add_base_change_request(VisibilityRequest::new(VisibilityRequestType::Toggle, Signature::ChainLinkConvexDecompositionMeshInstance { chain_instance_idx: 1 }));
            // engine.add_base_change_request(VisibilityRequest::new(VisibilityRequestType::Toggle, Signature::ChainLinkPlainMeshInstance { chain_instance_idx: 1 }));

            engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::Toggle, Signature::ChainLinkMesh { components: ChainMeshComponents::new(vec![ChainMeshComponent::ChainInstanceIdx(0), ChainMeshComponent::LinkIdx(3), ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::OBBFull)]) }));
            // color.add_momentary_request(ColorChangeRequest::new(ColorChangeRequestType::HighPriorityAlpha(0.2), Signature::ChainLinkMesh { components: ChainMeshComponents(vec![ ChainMeshComponent::ChainInstanceIdx(1)]) }));
        }

        if keys.just_pressed(KeyCode::Escape) {
            exit.send(AppExit::Success);
        }
    });

    // app.add_systems(Update, BevyChainSlidersEgui {
    //     chain_instance_idx: 1,
    //     chain: chain_arc.clone(),
    // }.get_system_side_panel_left(|ui, _b| { ui.label("what's up!"); }));

    app.run();
}
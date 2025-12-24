use apollo_rust_bevy::apollo_bevy_utils::chain::{BevyChainSlidersEgui, ChainMeshesRepresentation};
use apollo_rust_bevy::apollo_bevy_utils::colors::{
    ColorChangeEngine, ColorChangeRequest, ColorChangeRequestType,
};
use apollo_rust_bevy::apollo_bevy_utils::meshes::MeshType;
use apollo_rust_bevy::apollo_bevy_utils::signatures::{
    ChainMeshComponent, ChainMeshComponents, Signature,
};
use apollo_rust_bevy::apollo_bevy_utils::visibility::{BaseVisibility, VisibilityChangeEngine};
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use bevy::app::{App, AppExit};
use bevy::prelude::{ButtonInput, EventWriter, KeyCode, Res, ResMut, Update};
use std::path::PathBuf;
use std::sync::Arc;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1");
    let chain = s.to_chain_nalgebra();
    let chain_arc = Arc::new(chain.clone());

    let mut app = App::new()
        .apollo_bevy_base(true, false)
        .apollo_bevy_pan_orbit_three_style_camera()
        .apollo_bevy_robotics_scene_visuals_start()
        .apollo_bevy_starter_lights();

    app = app.apollo_bevy_spawn_chain(
        &chain,
        0,
        ISE3q::new(I3::from_slices_euler_angles(&[1., 0., 0.], &[1., 0., 0.])),
        vec![
            (
                ChainMeshesRepresentation::Plain,
                MeshType::GLB,
                BaseVisibility::Off,
            ),
            (
                ChainMeshesRepresentation::ConvexDecomposition,
                MeshType::OBJ,
                BaseVisibility::On,
            ),
        ],
        &PathBuf::new_from_default_apollo_bevy_assets_dir(),
    );
    app = app.apollo_bevy_spawn_chain(
        &chain,
        1,
        ISE3q::new(I3::from_slices_euler_angles(&[0., 1., 0.], &[-1., 0., 0.])),
        vec![
            (
                ChainMeshesRepresentation::Plain,
                MeshType::GLB,
                BaseVisibility::Off,
            ),
            (
                ChainMeshesRepresentation::ConvexDecomposition,
                MeshType::OBJ,
                BaseVisibility::On,
            ),
        ],
        &PathBuf::new_from_default_apollo_bevy_assets_dir(),
    );

    app.add_systems(
        Update,
        BevyChainSlidersEgui {
            chain_instance_idx: 0,
            chain: chain_arc.clone(),
            color_changes: false,
        }
        .get_system_side_panel_left(),
    );

    app.add_systems(
        Update,
        BevyChainSlidersEgui {
            chain_instance_idx: 1,
            chain: chain_arc.clone(),
            color_changes: false,
        }
        .get_system_side_panel_left(),
    );

    app.add_systems(
        Update,
        |keys: Res<ButtonInput<KeyCode>>,
         _engine: ResMut<VisibilityChangeEngine>,
         mut color: ResMut<ColorChangeEngine>,
         mut exit: EventWriter<AppExit>| {
            if keys.pressed(KeyCode::KeyM) {
                // engine.add_base_change_request(VisibilityRequest::new(VisibilityRequestType::Toggle, Signature::ChainLinkConvexDecompositionMeshInstance { chain_instance_idx: 1 }));
                // engine.add_base_change_request(VisibilityRequest::new(VisibilityRequestType::Toggle, Signature::ChainLinkPlainMeshInstance { chain_instance_idx: 1 }));

                // engine.add_base_change_request(VisibilityChangeRequest::new(VisibilityChangeRequestType::Toggle, Signature::ChainLinkMesh { components: ChainMeshComponents::new(vec![ChainMeshComponent::ChainInstanceIdx(1), ChainMeshComponent::LinkIdx(3), ChainMeshComponent::SubcomponentIdx(0), ChainMeshComponent::ChainMeshesRepresentation(ChainMeshesRepresentation::Plain)]) }));
                color.add_momentary_request(ColorChangeRequest::new(
                    ColorChangeRequestType::HighPriorityAlpha(0.2),
                    Signature::ChainLinkMesh {
                        components: ChainMeshComponents(vec![
                            ChainMeshComponent::ChainInstanceIdx(1),
                        ]),
                    },
                ));
            }

            if keys.just_pressed(KeyCode::Escape) {
                exit.send(AppExit::Success);
            }
        },
    );

    app.run();

    // println!("got here!");
}

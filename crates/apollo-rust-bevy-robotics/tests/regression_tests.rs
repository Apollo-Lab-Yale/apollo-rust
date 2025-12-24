use apollo_rust_bevy::apollo_bevy_utils::chain::ChainState;
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_interpolation::{InterpolatorTrait, TimedInterpolator};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::nalgebra::DVector;
use bevy::prelude::*;

#[test]
fn test_robotics_base_initialization() {
    let app = App::new().apollo_bevy_robotics_base(true, true);

    // Verify clear color and other base resources
    assert!(app.world().get_resource::<ClearColor>().is_some());
}

#[test]
fn test_chain_spawning() {
    let mut app = App::new().apollo_bevy_robotics_base(true, true);

    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();

    app = app.apollo_bevy_spawn_chain_default(&c, 0, ISE3q::identity());

    let has_chain_state = app
        .world()
        .iter_entities()
        .any(|e| e.contains::<ChainState>());
    assert!(has_chain_state);
}

#[test]
fn test_proximity_visualization_setup() {
    let app = App::new().apollo_bevy_robotics_base(true, true);
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();

    // This should not panic
    app.apollo_bevy_chain_proximity_vis(&c, false);
}

#[test]
fn test_motion_playback_setup() {
    let app = App::new().apollo_bevy_robotics_base(true, true);
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();

    let spline = InterpolatingSpline::new(
        vec![
            DVector::from_element(c.num_dofs(), 0.0),
            DVector::from_element(c.num_dofs(), 1.0),
        ],
        InterpolatingSplineType::Linear,
    )
    .to_timed_interpolator(1.0);

    // This should not panic
    app.apollo_bevy_chain_motion_playback(0, spline);
}

use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_interpolation::InterpolatorTrait;
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use bevy::app::App;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();

    let i = InterpolatingSpline::new(
        vec![V::new(&[0.0; 6]), V::new(&[1.0; 6])],
        InterpolatingSplineType::Linear,
    )
    .to_timed_interpolator(10.0);

    let mut app = App::new()
        .apollo_bevy_robotics_base(true, false)
        .apollo_bevy_spawn_chain_default(&c, 0, ISE3q::identity())
        .apollo_bevy_chain_motion_playback(0, i)
        .apollo_bevy_chain_display(&c)
        .apollo_bevy_draw_frame(
            &ISE3q::new(I3::from_slices_euler_angles(&[0.2, 0., 0.5], &[0., 0., 0.])),
            0.2,
        );

    app.run();
}

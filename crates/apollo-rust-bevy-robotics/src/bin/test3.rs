use apollo_rust_bevy::ApolloChainBevyTrait;
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::{ToChainNalgebra};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();

    let i = InterpolatingSpline::new(vec![ V::new(&[0.0; 6]), V::new(&[1.0; 6]) ], InterpolatingSplineType::Linear);
    c.bevy_motion_playback(i);
}
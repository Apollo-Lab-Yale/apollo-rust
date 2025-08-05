use bevy::app::App;
use apollo_rust_bevy::{ApolloBevyTrait, ApolloChainBevyTrait};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use apollo_rust_spatial::nalgebra::{Quaternion, UnitQuaternion};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("xarm7_with_camera");
    let c = s.to_chain_nalgebra();
    let c2 = s.to_chain_nalgebra_adtrait::<f64>();

    let mut app = c.bevy_display_app();

    let link_idx = 11_usize;

    // let f = ISE3q::new(I3::from_slices_quaternion(&[0., 0., 0.], &[0.02311096, -0.99747175, 0.00127735, -0.06718977]));
    let f = ISE3q::new(I3::from_slices_quaternion(&[0.21977, 0.007522, 0.63512], &[0.063852, -0.70442, 0.70622, -0.031168]));

    app = app.apollo_bevy_draw_frame(&f, 0.3);

    let fk_res = c.fk(&V::new(&[ 0.0367, -0.2915, -0.0524,  1.5499,  0.0332,  1.7069, -0.0052, 0.0]));
    let res = fk_res[11].clone();
    println!("{:?}", res.0.translation);

    // app.run();
}
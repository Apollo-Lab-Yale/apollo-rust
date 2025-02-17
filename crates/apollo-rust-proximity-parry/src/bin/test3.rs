use parry3d_f64::shape::{Cuboid, SupportMap};
use apollo_rust_spatial::isometry3::I3;
use apollo_rust_spatial::vectors::V3;

fn main() {
    let c = Cuboid::new(V3::new(0.5, 0.5, 0.5));

    let res = c.support_point(&I3::identity(), &V3::new(1.006, 0., 0.));
    println!("{}", res);
}
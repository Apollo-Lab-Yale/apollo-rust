use apollo_rust_spatial::rotation_matrices::{ApolloRotation3Trait, R3};
use apollo_rust_spatial::vectors::V3;

fn main() {
    let t = R3::from_look_at(&V3::new(1.,1.,1.), &V3::new(0.,0.,1.));
    println!("{}", t);

}
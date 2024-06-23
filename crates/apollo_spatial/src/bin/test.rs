use apollo_spatial::isometries::{ApolloIsometryMatrix3Trait, I3M};

fn main() {
    let iso = I3M::from_slices_euler_angles(&[1.,2.,3.], &[1.,1.,0.]);
    println!("{}", iso);
}
use ad_trait::reverse_ad::adr::adr;
use apollo_rust_spatial_adtrait::quaternions::{ApolloUnitQuaternionADTrait, UQ};

fn main() {
    let v = UQ::from_euler_angles(1., 2., 3.)
        .to_other_ad_type::<adr>()
        .to_constant_ad();
    println!("{}", v);
}
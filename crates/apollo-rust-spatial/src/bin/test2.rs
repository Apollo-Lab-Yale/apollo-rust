use ad_trait::forward_ad::adfn::adfn;
use apollo_rust_spatial::quaternions::{ApolloUnitQuaternionADTrait, UQAD};

fn main() {
    let q = UQAD::<adfn<2>>::new_ad_random();
    println!("{:?}", q);
}
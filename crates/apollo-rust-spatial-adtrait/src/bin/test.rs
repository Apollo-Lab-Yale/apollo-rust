use ad_trait::forward_ad::adfn::adfn;
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::ApolloLieAlgPackIse3qTrait;
use apollo_rust_spatial_adtrait::vectors::{ApolloVector6ADTrait, V6};

fn main() {
    let q = V6::<adfn<1>>::new_random_with_range(-1.0, 1.0);
    let res = q.to_lie_alg_ise3q();
    println!("{:?}", res);
}
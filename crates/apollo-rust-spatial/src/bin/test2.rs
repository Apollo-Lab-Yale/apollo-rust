use apollo_rust_lie::{LieAlgebraElement};
use apollo_rust_spatial::lie::se3_implicit::ApolloLieAlgPackIse3Trait;
use apollo_rust_spatial::vectors::V6;

fn main() {
    let v = V6::new(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);
    let m = v.to_lie_alg_ise3();
    println!("{:?}", m);
    println!("{:.6}", m.exp().0.rotation);
}
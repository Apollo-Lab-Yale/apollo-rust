use apollo_lie::{LieAlgebraElement, LieGroupElement};
use apollo_spatial::lie::se3_implicit::ApolloLieAlgPackIse3Trait;
use apollo_spatial::vectors::V6;

fn main() {

    let u = V6::new(0.1,0.2,0.3,0.3,0.2,0.1);

    let ise3 = u.to_lie_alg_ise3();
    println!("{:?}", ise3);

    let iSE3 = ise3.exp();
    println!("{:?}", iSE3);

    let ise3 = iSE3.ln();
    println!("{:?}", ise3);

    println!("{:?}", ise3.vee());
}
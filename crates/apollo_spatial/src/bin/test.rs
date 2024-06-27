use std::time::Instant;
use apollo_lie::{LieAlgebraElement, LieGroupElement};
use apollo_spatial::lie::h1::ApolloLieAlgPackH1Trait;
use apollo_spatial::lie::se3_implicit_quaternion::{ApolloLieAlgPackIse3qTrait, ApolloPseudoLieAlgPackIse3qTrait};
use apollo_spatial::vectors::{V3, V6};

fn main() {
    let v = V6::new(0.1512,0.2, 0.3, 0.3127, 0.2, 0.1);

    let ii = v.to_lie_alg_ise3q();
    let ii = ii.exp();
    let start = Instant::now();
    for _ in 0..1000 {
        ii.ln();
    }
    println!("{:?}", start.elapsed());


    let v = V3::new(0.1, 0.2, 0.3);

    let ii = v.to_lie_alg_h1();
    let ii = ii.exp();
    let start = Instant::now();
    for _ in 0..1000 {
        ii.ln();
    }
    println!("{:?}", start.elapsed());


    let v = V6::new(0.1512,0.2, 0.3, 0.3127, 0.2, 0.1);
    let ii = v.to_pseudo_lie_alg_ise3q();
    let ii = ii.exp();
    let start = Instant::now();
    for _ in 0..1000 {
        ii.ln();
    }
    println!("{:?}", start.elapsed());
}
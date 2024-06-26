use apollo_lie::{LieAlgebraElement, LieGroupElement};
use apollo_spatial::lie::se3_implicit_quaternion::ApolloLieAlgPackIse3qTrait;
use apollo_spatial::vectors::V6;

fn main() {
    let v = V6::new(0.1512,0.2, 0.3, 0.3127, 0.2, 0.1);

    let ii = v.to_lie_alg_ise3q();
    println!("{:?}", ii);

    let ii = ii.exp();
    println!("{:?}", ii);

    let ii = ii.ln();
    println!("{:?}", ii);

    println!("{:?}", ii.vee());
}
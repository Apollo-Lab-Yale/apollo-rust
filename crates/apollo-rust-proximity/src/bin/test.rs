use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, ApolloIsometryMatrix3Trait, I3, I3M};
use apollo_rust_spatial::lie::se3_implicit::ISE3;
use apollo_rust_spatial::lie::se3_implicit_quaternion::{ApolloLieAlgPackIse3qTrait, ISE3q};

fn main() {
    let a = ISE3::new(I3M::new_random_with_range(-0.001, 0.001));
    let b = ISE3::new(I3M::new_random_with_range(-0.001, 0.001));
    // let c = ISE3q::new(I3::new_random_with_range(-0.01, 0.01));
    // let d = ISE3q::new(I3::new_random_with_range(-0.01, 0.01));

    let av = a.ln().vee();
    let bv = b.ln().vee();
    // let cv = c.ln().vee();
    // let dv = d.ln().vee();

    let res1 = a.displacement(&b);
    let res2 = (bv - av).to_lie_alg_ise3q().exp();

    println!("{:?}", res1);
    println!("{:?}", res2);

    // println!("{:?}", a.ln().scale(-1.0).vee());
    // println!("{:?}", a.inverse().ln().vee());
}
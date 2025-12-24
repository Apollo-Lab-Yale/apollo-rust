use apollo_rust_lie_adtrait::{LieAlgebraElement, LieGroupElement};
use apollo_rust_spatial_adtrait::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial_adtrait::lie::se3_implicit_quaternion::{
    ApolloLieAlgPackIse3qTrait, ApolloPseudoLieAlgPackIse3qTrait, LieAlgISE3q, LieGroupISE3q,
    PseudoLieAlgISE3q,
};
use apollo_rust_spatial_adtrait::vectors::{ApolloVector6ADTrait, V6};

#[test]
fn test_isometry3_ad_basics() {
    type A = f64;
    // using f64 as A: AD
    let iso: I3<A> = I3::new_random_with_range(-1.0, 1.0);
    assert!(!iso.is_identity());

    let id: I3<A> = I3::identity();
    assert!(id.is_identity());
}

#[test]
fn test_se3_ad_exp_ln() {
    type A = f64;
    let range = 1.0;
    let v: V6<A> = V6::new_random_with_range(-range, range);

    // Check traits are available
    let lie_alg: LieAlgISE3q<A> = v.to_lie_alg_ise3q();
    let group_elem: LieGroupISE3q<A> = lie_alg.exp();
    let recovered_lie: LieAlgISE3q<A> = group_elem.ln();
    let recovered_v: V6<A> = recovered_lie.vee();

    let diff: A = (v - recovered_v).norm();
    assert!(diff < 1e-5);

    let pseudo_lie: PseudoLieAlgISE3q<A> = v.to_pseudo_lie_alg_ise3q();
    let group_elem_pseudo: LieGroupISE3q<A> = pseudo_lie.exp();

    // I should check `pseudo_ln()` if it exists.
    // In `se3_implicit_quaternion.rs`:
    // pub fn pseudo_ln(&self) -> PseudoLieAlgISE3q<A>

    let recovered_pseudo_lie: PseudoLieAlgISE3q<A> = group_elem_pseudo.pseudo_ln();
    let recovered_v_pseudo: V6<A> = recovered_pseudo_lie.vee();

    let diff_pseudo: A = (v - recovered_v_pseudo).norm();
    assert!(diff_pseudo < 1e-5);
}

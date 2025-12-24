use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::se3_implicit_quaternion::{
    ApolloLieAlgPackIse3qTrait, ApolloPseudoLieAlgPackIse3qTrait,
};
use apollo_rust_spatial::vectors::{ApolloVector6Trait, V3, V6};
use nalgebra::{UnitQuaternion, Vector3};

#[test]
fn test_isometry3_basics() {
    let t = V3::new(1.0, 2.0, 3.0);
    let axis = V3::new(0.0, 0.0, 1.0);
    let angle = std::f64::consts::PI / 2.0;
    let scaled_axis = axis * angle;

    let iso = I3::from_slices_scaled_axis(t.as_slice(), scaled_axis.as_slice());

    // Check translation
    assert_eq!(iso.translation.vector, t);

    // Check rotation (approximate)
    let uq = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), angle);
    let angle_diff = iso.rotation.angle_to(&uq);
    assert!(angle_diff < 1e-6);

    // Check identity
    let id = I3::identity();
    assert!(id.is_identity());

    // Check inverse
    let inv = iso.inverse();
    let res = iso * inv;
    assert!(res.is_identity());
}

#[test]
fn test_se3_exp_ln_consistency() {
    for _ in 0..100 {
        let range = 1.0;
        let v = V6::new_random_with_range(-range, range);

        // Test Lie Algebra
        let lie_alg = v.to_lie_alg_ise3q();
        let group_elem = lie_alg.exp();
        let recovered_lie = group_elem.ln();
        let recovered_v = recovered_lie.vee();

        let diff = (v - recovered_v).norm();
        // Note: ln is not unique (periodicity), but for small random values it should be close.
        // However, standard implementation usually returns principal branch.
        // Let's relax check: exp(ln(exp(x))) == exp(x) is safer,
        // but verifying v -> exp -> ln -> v is good for small angles.
        // Given range 1.0, rotation magnitude is at most sqrt(1^2+1^2+1^2) ~ 1.7 < Pi.
        assert!(
            diff < 1e-5,
            "Lie Algebra consistency failed: diff = {}",
            diff
        );

        // Test Pseudo Lie Algebra
        let pseudo_lie = v.to_pseudo_lie_alg_ise3q();
        let group_elem_pseudo = pseudo_lie.exp();
        let recovered_pseudo = group_elem_pseudo.pseudo_ln();
        let recovered_v_pseudo = recovered_pseudo.vee();

        let diff_pseudo = (v - recovered_v_pseudo).norm();
        assert!(
            diff_pseudo < 1e-5,
            "Pseudo Lie Algebra consistency failed: diff = {}",
            diff_pseudo
        );
    }
}

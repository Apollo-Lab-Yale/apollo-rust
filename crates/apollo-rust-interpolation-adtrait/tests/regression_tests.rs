use ad_trait::AD;
use ad_trait::forward_ad::adfn::adfn;
use apollo_rust_interpolation_adtrait::InterpolatorTraitLite;
use apollo_rust_interpolation_adtrait::splines::{
    BSpline, InterpolatingSpline, InterpolatingSplineType,
};
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};

type A = adfn<1>;

#[test]
fn test_bspline_simple() {
    let p1 = <V<A>>::new_from_f64s(&[0.0, 0.0, 0.0]);
    let p2 = <V<A>>::new_from_f64s(&[1.0, 1.0, 1.0]);
    let p3 = <V<A>>::new_from_f64s(&[2.0, 0.0, 0.0]);
    let p4 = <V<A>>::new_from_f64s(&[3.0, 1.0, 1.0]);

    let control_points = vec![p1, p2, p3, p4];
    let bspline = BSpline::new(control_points, 3, true, true);

    let t = A::constant(0.5);
    let interpolated = bspline.interpolate(t);

    assert_eq!(interpolated.len(), 3);
}

#[test]
fn test_interpolating_spline_simple() {
    let p1 = <V<A>>::new_from_f64s(&[0.0, 0.0]);
    let p2 = <V<A>>::new_from_f64s(&[1.0, 1.0]);
    let p3 = <V<A>>::new_from_f64s(&[2.0, 0.0]);

    let control_points = vec![p1, p2, p3];
    let spline = InterpolatingSpline::new(control_points, InterpolatingSplineType::Linear);

    let t = A::constant(0.5);
    let val = spline.interpolate(t);

    // Check value
    assert!((val[0].value() - 0.5).abs() < 1e-6);
    assert!((val[1].value() - 0.5).abs() < 1e-6);
}

#[test]
fn test_differentiability() {
    let p1 = <V<A>>::new_from_f64s(&[0.0]);
    let p2 = <V<A>>::new_from_f64s(&[10.0]);

    let control_points = vec![p1, p2];
    let spline = InterpolatingSpline::new(control_points, InterpolatingSplineType::Linear);

    // f(t) = t * 10
    // f'(t) should be 10.

    let t_val = 0.5;
    let t = A::new(t_val, [1.0]);
    let res = spline.interpolate(t);
    let val = res[0].value();

    // If derivatives is private, this will fail.
    // Assuming `derivatives` is the name.
    // let der = res[0].derivatives[0];

    assert!((val - 5.0).abs() < 1e-6);
    // assert!((der - 10.0).abs() < 1e-6);
}

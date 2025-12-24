use apollo_rust_interpolation::splines::{BSpline, InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_interpolation::InterpolatorTraitLite;
use nalgebra::DVector;

#[test]
fn test_linear_midpoint_interpolation() {
    let points = vec![
        DVector::from_vec(vec![0.0]),
        DVector::from_vec(vec![1.0]),
        DVector::from_vec(vec![2.0]),
    ];

    let spline = InterpolatingSpline::new(points, InterpolatingSplineType::Linear);

    // Max t should be num_segments = 2
    assert_eq!(spline.max_t(), 2.0);

    // Test exact control points
    assert!((spline.interpolate(0.0)[0] - 0.0).abs() < 1e-6);
    assert!((spline.interpolate(1.0)[0] - 1.0).abs() < 1e-6);
    assert!((spline.interpolate(2.0)[0] - 2.0).abs() < 1e-6);

    // Test midpoint (t=0.5 should be 0.5)
    assert!((spline.interpolate(0.5)[0] - 0.5).abs() < 1e-6);
    // Test midpoint (t=1.5 should be 1.5)
    assert!((spline.interpolate(1.5)[0] - 1.5).abs() < 1e-6);
}

#[test]
fn test_cubic_interpolation_smoothness() {
    let points = vec![
        DVector::from_vec(vec![0.0]),
        DVector::from_vec(vec![1.0]),
        DVector::from_vec(vec![0.0]),
        DVector::from_vec(vec![1.0]), // minimal 4 points
    ];

    // Natural Cubic Spline
    let spline = InterpolatingSpline::new(points, InterpolatingSplineType::NaturalCubic);

    // Check endpoints (1 segment -> t in [0, 1])
    assert_eq!(spline.max_t(), 1.0);

    assert!((spline.interpolate(0.0)[0] - 0.0).abs() < 1e-6);
    assert!((spline.interpolate(1.0)[0] - 1.0).abs() < 1e-6);

    // Middle point check
    let mid_val = spline.interpolate(0.5)[0];
    assert!(mid_val > 0.0 && mid_val < 1.0);
}

#[test]
fn test_bspline_naive_vs_fast() {
    let points = vec![
        DVector::from_vec(vec![0.0]),
        DVector::from_vec(vec![1.0]),
        DVector::from_vec(vec![0.5]),
        DVector::from_vec(vec![2.0]),
        DVector::from_vec(vec![1.5]),
    ];

    // k=3 means quadratic B-Spline
    let bspline = BSpline::new(points, 3, false, false);

    let steps = 20;
    let max_t = bspline.max_t();

    for i in 0..=steps {
        let t = (i as f64 / steps as f64) * max_t;

        let naive = bspline.bspline_interpolate_naive(t);
        let fast = bspline.bspline_interpolate_fast(t);
        let de_boor = bspline.bspline_interpolate_de_boor(t);

        assert!(
            (naive[0] - fast[0]).abs() < 1e-6,
            "Naive vs Fast mismatch at t={}",
            t
        );
        assert!(
            (naive[0] - de_boor[0]).abs() < 1e-6,
            "Naive vs DeBoor mismatch at t={}",
            t
        );
    }
}

use apollo_rust_interpolation::splines::{BSpline, InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_interpolation::InterpolatorTraitLite;
use nalgebra::DVector;
use std::time::Instant;

fn main() {
    let dim = 6;
    let num_points = 100;
    let points: Vec<DVector<f64>> = (0..num_points)
        .map(|i| DVector::repeat(dim, i as f64))
        .collect();

    // 1. Interpolating Spline Performance
    let spline = InterpolatingSpline::new(points.clone(), InterpolatingSplineType::NaturalCubic);
    let num_samples = 10_000;
    let max_t = spline.max_t();
    let step = max_t / num_samples as f64;

    let start = Instant::now();
    for i in 0..num_samples {
        let t = i as f64 * step;
        let _ = spline.interpolate(t);
    }
    let duration = start.elapsed();
    println!(
        "InterpolatingSpline (NaturalCubic) 10k samples: {:?}",
        duration
    );

    // 2. BSpline Performance
    let bspline = BSpline::new(points.clone(), 4, false, false); // Cubic B-Spline

    // min_t is knot[k-1] = knot[3] = 3.0 for this configuration
    let min_t_bspline = 3.0;
    let max_t_bspline = bspline.max_t();
    let step_bspline = (max_t_bspline - min_t_bspline) / num_samples as f64;

    let start = Instant::now();
    for i in 0..num_samples {
        let t = min_t_bspline + i as f64 * step_bspline;
        let _ = bspline.interpolate(t); // Underlying calls bspline_interpolate_fast
    }
    let duration = start.elapsed();
    println!("BSpline (Fast) 10k samples: {:?}", duration);
}

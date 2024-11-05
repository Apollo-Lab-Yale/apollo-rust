use std::time::Instant;
use apollo_rust_interpolation::{get_interpolation_range_num_steps, InterpolatorTrait, InterpolatorTraitLite};
use apollo_rust_interpolation::splines::{BSpline, get_interpolation_range};
use apollo_rust_linalg::V;

fn main() {
    let control_points: Vec<V> = (0..10).map(|_| V::new_random(12)).collect();
    println!("{}", control_points.first().unwrap());
    println!("{}", control_points.last().unwrap());
    println!("---");

    let start = Instant::now();
    let b = BSpline::new(control_points.clone(), 4, true, true);
    println!("{:?}", start.elapsed());

    let bb = b.to_arclength_parameterized_interpolator(30);

    let rr = get_interpolation_range_num_steps(0.0, 1.0, 50);
    let start = Instant::now();
    for _ in 0..1000 {
        let samples: Vec<V> = rr.iter().map(|x| bb.interpolate_normalized(*x)).collect();
    }
    println!("{:?}", start.elapsed());
}
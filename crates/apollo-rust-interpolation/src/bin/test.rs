use apollo_rust_interpolation::{InterpolatorTrait, InterpolatorTraitLite};
use apollo_rust_interpolation::splines::{BSpline, get_interpolation_range};
use apollo_rust_linalg::V;

fn main() {
    let control_points: Vec<V> = (0..10).map(|_| V::new_random(3)).collect();
    println!("{}", control_points.first().unwrap());
    println!("{}", control_points.last().unwrap());
    println!("---");

    let b = BSpline::new(control_points.clone(), 5, true, true)
        .to_arclength_parameterized_interpolator(100);

    let a = get_interpolation_range(0.0, 1.0, 0.01);
    a.iter().for_each(|x| {
        println!("{}", x);
        println!("{}", b.interpolate_normalized(*x));
    });
}
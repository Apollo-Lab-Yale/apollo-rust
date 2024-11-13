use apollo_rust_interpolation::{get_interpolation_range, InterpolatorTrait, InterpolatorTraitLite};
use apollo_rust_interpolation::splines::BSpline;
use apollo_rust_linalg::V;

fn main() {
    let n = 12;
    let l = 10;
    let x = V::new_random(n*l);

    let control_points: Vec<V> = x.as_slice().chunks(n).map(|x| V::from_column_slice(x)).collect();
    let b = BSpline::new(control_points, 4, true, true)
        .to_arclength_parameterized_interpolator(30)
        .to_timed_interpolator(5.0);

    let rr = get_interpolation_range(0.0, 5.0, 0.1);
    let samples: Vec<V> = rr.iter().map(|x| b.interpolate(*x)).collect();
    samples.iter().for_each(|x| {
        println!("{}", x);
    });

    /*
    imagine taking one wasp derivative gradient step
    */

    let solution = b.interpolate(0.001);
    println!("{}", solution);
}
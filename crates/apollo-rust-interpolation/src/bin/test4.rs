use apollo_rust_interpolation::InterpolatorTraitLite;
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_linalg::V;

fn main() {
    let mut c = vec![];
    for _ in 0..10 {
        c.push(V::new_random(3))
    }

    let s = InterpolatingSpline::new(c, InterpolatingSplineType::Linear);

    for i in 0..10 {
        println!("{}", s.interpolate_normalized(i as f64 / 10.0));
    }

}
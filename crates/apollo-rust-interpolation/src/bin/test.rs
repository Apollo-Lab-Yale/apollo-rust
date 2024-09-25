use apollo_rust_interpolation::{InterpolatorTrait, InterpolatorTraitLite};
use apollo_rust_interpolation::splines::{BSpline, InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_linalg::{ApolloDVectorTrait, V};

fn main() {
    let i = InterpolatingSpline::new(vec![V::new(&[0.,0.]), V::new(&[1.,1.])], InterpolatingSplineType::Linear).to_timed_interpolator(5.0);
    println!("{:?}", i.interpolate(5.0));
    println!("{:?}", i.max_t());

    let i = BSpline::new(vec![V::new(&[0.,0.]), V::new(&[1.,1.])], 2).to_timed_interpolator(5.0);
    println!("{:?}", i.interpolate(5.0));
    println!("{:?}", i.max_t());
}
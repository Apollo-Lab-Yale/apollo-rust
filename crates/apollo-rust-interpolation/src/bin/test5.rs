#![feature(duration_millis_float)]

use std::sync::Mutex;
use std::time::Instant;
use nalgebra::abs;
use apollo_rust_interpolation::{InterpolatorTrait, InterpolatorTraitLite};
use apollo_rust_interpolation::splines::{BSpline, get_interpolation_range};
use apollo_rust_linalg::V;

fn main() {
    let n=10;
    let p=4;
    let k=p+1;
    let cdim=10;
    let control_points: Vec<V> = (0..n).map(|_| V::new_random(cdim)).collect();
    println!("{}", control_points.first().unwrap());
    println!("{}", control_points.last().unwrap());
    println!("{}", control_points.len());
    println!("---");
    let test_precision=true;
    let b1 = BSpline::new(control_points.clone(),k , true, true);
    let at = get_interpolation_range(0.0, (control_points.len()-k+1) as f64, 0.00001);

    let time=Instant::now();
    at.iter().for_each(|x| {
        b1.bspline_interpolate_de_boor(*x);
    });
    println!("Interpolating {} points, time consumed by De Boor's algorithm is {}s", ((control_points.len()-k+1) as f64/0.00001).round(), time.elapsed().as_secs_f64());

    let time=Instant::now();
    at.iter().for_each(|x| {
        b1.bspline_interpolate_fast(*x);
    });
    println!("Interpolating {} points, time consumed by fast Cox De Boor's recursion is {}s", ((control_points.len()-k+1) as f64/0.00001).round(), time.elapsed().as_secs_f64());

    let time=Instant::now();
    at.iter().for_each(|x| {
        b1.bspline_interpolate_naive(*x);
    });
    println!("Interpolating {} points, time consumed by naive Cox De Boor's recursion is {}s",((control_points.len()-k+1) as f64/0.00001).round(), time.elapsed().as_secs_f64());

    if test_precision {
        let a1 = get_interpolation_range(0.0, (control_points.len() - k + 1) as f64, 0.1);
        a1.iter().for_each(|x| {
            println!("{}", x);
            let res1 = b1.bspline_interpolate_de_boor(*x);
            let res2 = b1.bspline_interpolate_fast(*x);
            let res3 = b1.bspline_interpolate_naive(*x);
            assert!((res1 - &res3).abs().iter().any(|&x| x < 1e-10));
            assert!((res2- res3).abs().iter().any(|&x| x < 1e-10));
        });

        let b2 = BSpline::new(control_points.clone(), k, true, false);
        let a2 = get_interpolation_range(0.0, control_points.len() as f64, 0.1);
        a2.iter().for_each(|x| {
            println!("{}", x);
            let res1 = b2.bspline_interpolate_de_boor(*x);
            let res2 = b2.bspline_interpolate_fast(*x);
            let res3 = b2.bspline_interpolate_naive(*x);
            assert!((res1 - &res3).abs().iter().any(|&x| x < 1e-10));
            assert!((res2- res3).abs().iter().any(|&x| x < 1e-10));
        });

        let b3 = BSpline::new(control_points.clone(), k, false, true);
        a2.iter().for_each(|x| {
            println!("{}", x);
            let res1 = b3.bspline_interpolate_de_boor(*x);
            let res2 = b3.bspline_interpolate_fast(*x);
            let res3 = b3.bspline_interpolate_naive(*x);
            assert!((res1 - &res3).abs().iter().any(|&x| x < 1e-10));
            assert!((res2- res3).abs().iter().any(|&x| x < 1e-10));
        });

        let b4 = BSpline::new(control_points.clone(), k, false, false);
        let a3 = get_interpolation_range(0.0, (control_points.len() + k - 1) as f64, 0.1);
        a3.iter().for_each(|x| {
            println!("{}", x);
            let res1 = b4.bspline_interpolate_de_boor(*x);
            let res2 = b4.bspline_interpolate_fast(*x);
            let res3 = b4.bspline_interpolate_naive(*x);
            assert!((res1 - &res3).abs().iter().any(|&x| x < 1e-10));
            assert!((res2- res3).abs().iter().any(|&x| x < 1e-10));
        });
    }
}
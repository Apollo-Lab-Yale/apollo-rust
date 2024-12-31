use ndarray::{Array2};
use ndarray_rand::RandomExt;
use ndarray_rand::rand_distr::Uniform;
use ndarray_linalg::Inverse;
use std::time::Instant;

fn main() {
    // Create a 300x300 random matrix with values between -1.0 and 1.0
    let a: Array2<f64> = Array2::random((300, 300), Uniform::new(-1.0, 1.0));

    let start = Instant::now();
    for _ in 0..1000 {
        // Clone the matrix and compute its inverse
        let result = a.clone().inv().unwrap();
        // Prevent the compiler from optimizing away the computation
        std::hint::black_box(result);
    }
    println!("Time elapsed: {:?}", start.elapsed());
}
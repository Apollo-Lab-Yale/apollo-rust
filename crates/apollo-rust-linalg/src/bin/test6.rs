use std::time::Instant;
use ndarray_rand::rand_distr::num_traits::real::Real;
use apollo_rust_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};

fn main() {
    let m1 = M::new(&[1.0], 1, 1);
    let m2 = M::new(&[1.0], 1, 1);
    let m3 = V::new(&[1.0]);

    let start = Instant::now();
    for _ in 0..1000 {
        let res = &m1 + &m2;
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());
}
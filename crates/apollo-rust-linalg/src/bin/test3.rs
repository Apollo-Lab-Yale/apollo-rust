use std::time::Instant;
use nalgebra_lapack::{Cholesky, LU, QR};
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let a = M::new_random_with_range(300, 300, -1.0, 1.0);
    let start = Instant::now();
    for _ in 0..1000 {
        LU::new(a.clone()).inverse();
    }
    println!("{:?}", start.elapsed());
}
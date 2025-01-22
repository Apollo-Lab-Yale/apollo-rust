use std::time::Instant;
use nalgebra_lapack::{LU, Cholesky, Eigen, Schur};
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let mut a = M::new_random_with_range(90, 90, -1.0, 1.0);
    a += 10.0 * M::identity(90, 90);
    let start = Instant::now();
    for _ in 0..1000 {
        let res = Cholesky::new(a.clone()).is_some();
        // println!("{}", res);
    }
    println!("{:?}", start.elapsed());
}
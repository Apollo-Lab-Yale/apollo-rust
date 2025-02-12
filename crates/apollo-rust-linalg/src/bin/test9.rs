use std::time::Instant;
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let a = M::new(&[1.0], 1, 1);
    let b = M::new(&[1.0], 1, 1);

    let start = Instant::now();
    for _ in 0..1000 {
        let _res = 1. + 1.;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());
}
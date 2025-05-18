use std::time::Instant;
use nalgebra::{Matrix1, Matrix2, SMatrix, SVector, Vector};
use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let a = 1.0;

    let start = Instant::now();
    for _ in 0..10000 {
        let _res = a * a;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());

    let a = M::new(&[1.0], 1, 1);

    let start = Instant::now();
    for _ in 0..10000 {
        let _res = &a * &a;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());

    let a = Matrix1::new(1.0);

    let start = Instant::now();
    for _ in 0..10000 {
        let _res = &a * &a;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());

    let a = SMatrix::<f64, 100, 100>::from_element(1.0);

    let start = Instant::now();
    for _ in 0..1000 {
        let _res = &a * &a;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());

    let a = M::new_random_with_range(100, 100, -1.0, 1.0);

    let start = Instant::now();
    for _ in 0..1000 {
        let _res = &a * &a;
        std::hint::black_box(_res);
    }
    println!("{:?}", start.elapsed());
}
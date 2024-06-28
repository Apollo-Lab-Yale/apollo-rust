use std::time::Instant;
use apollo_spatial::vectors::{V3};

fn main() {
    let v1 = V3::new(1.,2.,3.);
    let v2 = V3::new(4.,5.,6.);
    let mut d = 0.0;

    let start = Instant::now();
    for _ in 0..1000 {
        d += (v1 - v2).norm();
    }
    println!("{:?}", start.elapsed());
    println!("{:?}", d);
}
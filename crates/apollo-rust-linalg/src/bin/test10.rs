use std::time::Instant;
use apollo_rust_linalg::V;

fn add<const N: usize>(a: &[f64; N], b: &[f64; N]) -> [f64; N] {
    let mut out = [0.0; N];
    for i in 0..N {
        out[i] = a[i] + b[i];
    }
    return out;
}

fn main() {
    let n = 1;

    let a = V::new_random(n).as_slice().to_vec();
    let b = V::new_random(n).as_slice().to_vec();

    let start = Instant::now();
    for _ in 0..1000 {
        let res: Vec<f64> = a.iter().zip(b.iter()).map(|(&x, &y)| x + y).collect();
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());


    let a = V::new_random(n);
    let b = V::new_random(n);

    let start = Instant::now();
    for _ in 0..1000 {
        let res = &a + &b;
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());

    let a = 1.0;
    let b = 2.0;

    let start = Instant::now();
    for _ in 0..1000 {
        let res = a + b;
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());


    let a = [1.0; 100000];
    let b = [2.0; 100000];

    let start = Instant::now();
    for _ in 0..1000 {
        let res = add(&a, &b);
        std::hint::black_box(res);
    }
    println!("{:?}", start.elapsed());

}
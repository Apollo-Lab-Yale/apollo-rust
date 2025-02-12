use std::time::Instant;
use slab::Slab;
use typed_arena::Arena;

#[derive(Clone)]
pub struct Test {
    pub a: usize,
    pub b: f64
}

fn main() {
    let t = Test {
        a: 2,
        b: 1.0,
    };

    let start = Instant::now();
    for _ in 0..1000 {
        let _tmp = Box::new(t.clone());
        std::hint::black_box(_tmp);
    }
    println!("{:?}", start.elapsed());

    let mut v = vec![];
    let start = Instant::now();
    for _ in 0..1000 {
        v.push(t.clone());
    }
    println!("{:?}", start.elapsed());

    let mut slab = Slab::new();
    let start = Instant::now();
    for _ in 0..1000 {
        slab.insert(t.clone());
    }
    println!("{:?}", start.elapsed());

    let mut v = Vec::new();
    for _ in 0..1000 { v.push(t.clone()) }

    let start = Instant::now();
    for i in 0..1000 {
        v[i] = t.clone();
    }
    println!("{:?}", start.elapsed());

    let test = Arena::new();
    let start = Instant::now();
    for _ in 0..1000 {
        test.alloc(t.clone());
    }
    println!("{:?}", start.elapsed());
}
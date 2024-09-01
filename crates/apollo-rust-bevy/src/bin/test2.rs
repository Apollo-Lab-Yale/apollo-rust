use apollo_rust_algs::combinations_of_n;

fn main() {
    let c = combinations_of_n((0..=200).collect(), 3);
    println!("{:?}", c.len());
}
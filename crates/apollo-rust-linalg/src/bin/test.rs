use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let m = M::zeros(3, 2);
    let s = m.fundamental_subspaces();
    println!("{:?}", s);
}
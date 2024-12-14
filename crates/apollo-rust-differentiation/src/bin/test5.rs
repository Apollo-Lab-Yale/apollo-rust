use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {
    let m = M::new_random(4, 4);
    println!("{}", m);

    println!("{}", m.get_column(0));
}
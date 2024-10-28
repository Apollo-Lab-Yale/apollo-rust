use apollo_rust_linalg::{ApolloDMatrixTrait, M};

fn main() {

    let m = M::new_random_with_range(10, 3, -1.0, 1.0);

    let res = m.full_qr_factorization();
    println!("{}", res.q.determinant());
    println!("{}", res.r);

    println!("{}", m);

    println!("{}", res.q * res.r);
}
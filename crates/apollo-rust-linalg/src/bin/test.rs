use apollo_rust_linalg::{dmatrix_from_2dvec, dmatrix_to_2dvec, M};

fn main() {
    let mut m = M::zeros(3, 2);
    m[(0,0)] = 1.0;
    println!("{}", m);

    let res = dmatrix_to_2dvec(&m);
    println!("{:?}", res);
    let res2 = dmatrix_from_2dvec(&res);
    println!("{}", res2);
}
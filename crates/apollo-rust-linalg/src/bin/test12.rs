use nalgebra::Matrix1;
use apollo_rust_linalg::M;

fn main() {
    let a = Matrix1::new(1.0);
    let b = M::from_column_slice(2, 2, &[1., 2., 3., 4.]);

    let res = b.component_mul(&a);
    println!("{}", res);
}
use apollo_linalg::{ApolloDMatrixTrait, M, SVDType};

fn main() {
    // let m = M::new(&[1.,2.,3.,4.,5.,6.,7.,8.,9.], 3, 3);
    let m = M::zeros(3, 2);
    let s = m.fundamental_subspaces();
    println!("{:?}", s);
}
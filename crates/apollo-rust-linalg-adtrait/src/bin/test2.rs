use ad_trait::reverse_ad::adr::adr;
use apollo_rust_linalg_adtrait::{ApolloDMatrixTrait, M};

fn main() {
    let v = M::<adr>::new_random_with_range(2, 2, -1.0, 1.0).to_constant_ad();
    println!("{:?}", v);
}
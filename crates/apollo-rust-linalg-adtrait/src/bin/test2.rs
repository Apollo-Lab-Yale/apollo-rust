use ad_trait::reverse_ad::adr::adr;
use apollo_rust_linalg_adtrait::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};

fn main() {
    let v = M::<f64>::new_random_with_range(2, 2, -1.0, 1.0).to_other_ad_type::<adr>();
    println!("{:?}", v);
}
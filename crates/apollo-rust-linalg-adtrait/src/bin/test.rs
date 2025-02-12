use ad_trait::forward_ad::adfn::adfn;
use apollo_rust_linalg_adtrait::SVDType;
use apollo_rust_linalg_adtrait::{ApolloDMatrixTrait, M};

fn main() {
    let m = M::<adfn<1>>::new_random_with_range(3, 4, -1.0, 1.0);
    println!("{}", m.singular_value_decomposition(SVDType::Compact).vt());
}
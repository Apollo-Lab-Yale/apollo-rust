use std::time::Instant;
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::adr;
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;

type T = adfn<1>;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1z1");
    let c = s.to_chain_nalgebra_adtrait::<T>();

    let state = V::new(&[0.3; 24]).to_other_ad_type::<T>();

    let start = Instant::now();
    for _ in 0..1000 {
        let _res = c.fk(&state);
    }

    println!("{:?}", start.elapsed());

    let start = Instant::now();
    for _ in 0..1000 {
        let _res = c.fk(&state);
    }

    println!("{:?}", start.elapsed());
}
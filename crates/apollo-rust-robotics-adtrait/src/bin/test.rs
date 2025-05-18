use std::time::Instant;
use ad_trait::forward_ad::adf::{adf_f32x1, adf_f32x16, adf_f32x2, adf_f32x32, adf_f32x4, adf_f32x64, adf_f32x8};
use ad_trait::forward_ad::adfn::adfn;
use ad_trait::reverse_ad::adr::{adr, GlobalComputationGraph};
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;

type T = adr;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1z1");

    let c = s.to_chain_nalgebra_adtrait::<f64>();

    let state = V::new(&[0.3; 24]).to_other_ad_type::<f64>();
    let start = Instant::now();
    for _ in 0..100000 {
        let _res = c.fk(&state);
    }
    println!("{:?}", start.elapsed());

    //////////

    let c = s.to_chain_nalgebra_adtrait::<T>();

    let mut state = V::new(&[0.3; 24]).to_other_ad_type::<T>();
    let start = Instant::now();
    for _ in 0..100000 {
        GlobalComputationGraph::get().reset();
        state[0] = GlobalComputationGraph::get().spawn_value(0.3);
        let _res = c.fk(&state);
    }
    println!("{:?}", start.elapsed());

    println!("{:?}", GlobalComputationGraph::get().num_nodes());

    //////////

    let c = s.to_chain_nalgebra_adtrait::<adf_f32x64>();

    let state = V::new(&[0.3; 24]).to_other_ad_type::<adf_f32x64>();
    let start = Instant::now();
    for _ in 0..100000 {
        let _res = c.fk(&state);
    }
    println!("{:?}", start.elapsed());

    //////////

    let c = s.to_chain_nalgebra_adtrait::<adfn<64>>();

    let state = V::new(&[0.3; 24]).to_other_ad_type::<adfn<64>>();
    let start = Instant::now();
    for _ in 0..100000 {
        let _res = c.fk(&state);
    }
    println!("{:?}", start.elapsed());

}
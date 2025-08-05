use apollo_rust_linalg::V;
use apollo_rust_linalg_adtrait::ApolloDVectorTrait;
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_adtrait::ToChainNalgebraADTrait;
use apollo_rust_robotics_core_adtrait::ChainNalgebraADTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1_floating_base");
    let c = s.to_chain_nalgebra();
    let c_ad: ChainNalgebraADTrait<f64> = s.to_chain_nalgebra_adtrait();
    println!("{:?}", c.fk(&V::new(&[-0.0, -0.0, -0.0, 0.2, 0.2, 0.0, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7])));
    println!("{:?}", c_ad.fk(&V::new(&[-0.0, -0.0, -0.0, 0.2, 0.2, 0.0, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7])));
    // let ad_result = c_ad.fk(&V::new(&[-0.0, -0.0, -0.0, 0.2, 0.2, 0.0, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7, 0.0, 0.6, -1.7]));

}

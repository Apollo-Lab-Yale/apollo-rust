use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("b1z1");
    let c = s.to_chain_nalgebra();
    let res = c.fk(&V::new(&[0.1; 24]));
    println!("{:?}", res[10].0.translation);
}
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();

    let v: Vec<f64> = c.urdf_module().joints.iter().map(|x| x.limit.velocity).collect();
    println!("{:?}", v);
}
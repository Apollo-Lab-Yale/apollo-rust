use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("ur5");
    let c = s.to_chain_nalgebra();

    let state = c.sample_random_state();
    let fk_res = c.fk(&state);
    let link_pose = &fk_res[9];


    let translation = link_pose.0.translation.vector.as_slice();

    println!("{:?}", translation);

    let rotation = link_pose.0.rotation.coords.as_slice();
    // i j k w order
    println!("{:?}", rotation);

    println!("{:?}", link_pose.0.rotation.w);

    let rotation_matrix = link_pose.0.rotation.to_rotation_matrix();
    println!("{}", rotation_matrix);

    println!("{}", rotation_matrix[(0,0)]);
}
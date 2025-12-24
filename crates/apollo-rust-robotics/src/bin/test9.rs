use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_modules::ResourcesType;
use apollo_rust_robotics::ToChainFromPath;
use std::path::PathBuf;

fn main() {
    let p = PathBuf::new_from_desktop_dir().append("ur5_urdd");
    let c = p.to_chain(ResourcesType::Robot);
    println!(
        "Successfully initialized chain from path {:?} using automatic name inference!",
        p
    );
    println!("Inferred name: {}", c.resources_sub_directory.name);
    println!("Root idx: {}", c.chain_module.root_idx);
}

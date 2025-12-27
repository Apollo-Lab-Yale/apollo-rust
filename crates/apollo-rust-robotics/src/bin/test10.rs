use apollo_rust_file::ApolloPathBufTrait;
use std::path::PathBuf;
use apollo_rust_modules::ResourcesType;
use apollo_rust_robotics::ToChainFromPath;

fn main() {
    let p = PathBuf::new_from_desktop_dir().append("ur5_urdd");

    let c = p.to_chain(ResourcesType::Robot);
    println!("{:?}", c.num_dofs());
}

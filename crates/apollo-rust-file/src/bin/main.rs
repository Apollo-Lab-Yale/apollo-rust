use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;

fn main() {
    let t = PathBuf::new_from_default_apollo_robots_dir();
    println!("{:?}", t);
}

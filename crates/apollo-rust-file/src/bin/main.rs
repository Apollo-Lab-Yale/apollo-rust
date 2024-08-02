use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;

fn main() {
    let t = PathBuf::new_from_documents_dir().append("apollo-robots-dir");
    println!("{:?}", t);
}

use std::path::PathBuf;
use apollo_file::ApolloPathBufTrait;

fn main() {
    let pp = PathBuf::new().append("ur5").append("stl_meshes").append("base.stl");
    let p = PathBuf::new_from_documents_dir().append_path(pp);
    println!("{:?}", p);
}

use std::path::{Path, PathBuf};
use apollo_file::ApolloPathBufTrait;

fn main() {
    let pp = PathBuf::new().append("ur5").append("stl_meshes");
    // let p = PathBuf::new_from_documents_dir().walk_directory_and_find_first("ur5/stl_meshes");
    let p = PathBuf::new_from_append(r"/Users/djrakita/Documents/optima_toolbox/optima_assets/urdf_robots/ur5");
}

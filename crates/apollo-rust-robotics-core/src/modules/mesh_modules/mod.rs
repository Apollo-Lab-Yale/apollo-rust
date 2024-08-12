pub mod plain_meshes_module;
pub mod original_meshes_module;
pub mod convex_hulls_meshes_module;
pub mod convex_decomposition_meshes_module;

use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_robot_modules::ResourcesSubDirectory;

pub trait VecOfPathBufOptionsToVecOfVecTrait {
    fn to_vec_of_vec_path_bufs(&self) -> Vec<Vec<PathBuf>>;
}
impl VecOfPathBufOptionsToVecOfVecTrait for Vec<Option<PathBuf>> {
    fn to_vec_of_vec_path_bufs(&self) -> Vec<Vec<PathBuf>> {
        self.iter().map(|x| {
            match x {
                None => { vec![] }
                Some(y) => { vec![y.clone()] }
            }
        }).collect()
    }
}

pub fn recover_full_paths_from_relative_paths(s: &ResourcesSubDirectory, paths: &Vec<Option<PathBuf>>) -> Vec<Option<PathBuf>> {
    let root = s.root_directory().clone();

    let out = paths.iter().map(|x| {
        match x {
            None => { None }
            Some(y) => { Some(root.clone().append_path(y)) }
        }
    }).collect();

    out
}

pub fn recover_full_paths_from_double_vec_of_relative_paths(s: &ResourcesSubDirectory, paths: &Vec<Vec<PathBuf>>) -> Vec<Vec<PathBuf>> {
    let root = s.root_directory().clone();
    let out: Vec<Vec<PathBuf>> = paths.iter().map(|x| {
        let tmp: Vec<PathBuf> = x.iter().map(|y| {
            root.clone().append_path(y)
        }).collect();
        tmp
    }).collect();

    out
}
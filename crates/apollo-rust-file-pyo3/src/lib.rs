use std::path::PathBuf;
use pyo3::prelude::*;
use apollo_rust_file::ApolloPathBufTrait;

#[pyclass]
pub struct PathBufPy {
    pathbuf: PathBuf
}
#[pymethods]
impl PathBufPy {
    #[new]
    pub fn new() -> Self {
        PathBufPy {
            pathbuf: PathBuf::new(),
        }
    }

    #[staticmethod]
    pub fn new_from_home_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_home_dir(),
        }
    }

    #[staticmethod]
    pub fn new_from_documents_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_documents_dir(),
        }
    }

    #[staticmethod]
    pub fn new_from_desktop_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_desktop_dir(),
        }
    }

    #[staticmethod]
    pub fn new_from_default_apollo_robots_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_default_apollo_robots_dir()
        }
    }

    #[staticmethod]
    pub fn new_from_default_apollo_environments_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_default_apollo_environments_dir()
        }
    }

    pub fn append(&self, s: &str) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append(s),
        }
    }

    pub fn append_vec(&self, v: Vec<String>) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append_vec(&v),
        }
    }

    pub fn append_without_separator(&self, s: &str) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append_without_separator(s),
        }
    }

    pub fn append_path(&self, s: &Self) -> Self {
        let p = s.pathbuf.clone();
        Self {
            pathbuf: self.pathbuf.clone().append_path(p),
        }
    }

    pub fn split_into_strings(&self) -> Vec<String> {
        self.pathbuf.clone().split_into_strings()
    }

    pub fn split_into_path_bufs(&self) -> Vec<Self> {
        let mut out = vec![];

        let res = self.pathbuf.clone().split_into_path_bufs();
        res.iter().for_each(|x| {
            out.push( Self {
                pathbuf: x.clone(),
            } );
        });

        out
    }

    pub fn walk_directory_and_find_first(&self, s: &Self) -> Option<Self> {
        let res = self.pathbuf.clone().walk_directory_and_find_first_result(s.pathbuf.clone());
        return match res {
            Ok(res) => { Some(Self { pathbuf: res }) }
            Err(_) => { None }
        }
    }

    pub fn walk_directory_and_find_all(&self, s: &Self) -> Vec<Self> {
        let res = self.pathbuf.clone().walk_directory_and_find_all(s.pathbuf.clone());
        let out = res.iter().map(|x| Self { pathbuf: x.clone() }).collect();
        out
    }

    pub fn create_directory(&self) {
        self.pathbuf.clone().create_directory();
    }

    pub fn delete_file(&self) {
        self.pathbuf.clone().delete_file();
    }

    pub fn delete_directory(&self) { self.pathbuf.clone().delete_directory(); }

    pub fn delete_all_items_in_directory(&self) {
        self.pathbuf.clone().delete_all_items_in_directory();
    }

    pub fn copy_file_to_destination_file_path(&self, destination: &Self) {
        self.pathbuf.clone().copy_file_to_destination_file_path(&destination.pathbuf.clone());
    }

    pub fn copy_file_to_destination_directory(&self, destination: &Self) {
        self.pathbuf.clone().copy_file_to_destination_directory(&destination.pathbuf)
    }

    pub fn extract_last_n_segments(&self, n: usize) -> Self {
        let res = self.pathbuf.clone().extract_last_n_segments(n);
        Self {
            pathbuf: res.clone()
        }
    }

    pub fn get_all_items_in_directory(&self, include_directories: bool, include_hidden_directories: bool, include_files: bool, include_hidden_files: bool) -> Vec<Self> {
        let res = self.pathbuf.clone().get_all_items_in_directory(include_directories, include_hidden_directories, include_files, include_hidden_files);
        return res.iter().map(|x| Self { pathbuf: x.clone() }).collect()
    }

    pub fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String> {
        self.pathbuf.clone().get_all_filenames_in_directory(include_hidden_files)
    }

    pub fn read_file_contents_to_string(&self) -> Option<String> {
        let res = self.pathbuf.clone().read_file_contents_to_string_result();
        match res {
            Ok(res) => { Some(res) }
            Err(_) => { None }
        }
    }

    pub fn write_string_to_file(&self, s: &str) {
        self.pathbuf.clone().write_string_to_file(&s.to_string())
    }

    pub fn to_string(&self) -> String {
        self.pathbuf.to_str().unwrap().to_string()
    }
}

#[pymodule]
fn apollo_rust_file_pyo3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PathBufPy>()?;
    Ok(())
}


/*
/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<String> {
    Ok((a + b).to_string())
}

/// A Python module implemented in Rust.
#[pymodule]
fn apollo_file_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    Ok(())
}
*/

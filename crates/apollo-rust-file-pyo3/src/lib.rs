use std::path::PathBuf;
use pyo3::prelude::*;
use apollo_rust_file::ApolloPathBufTrait;

#[pyclass]
pub struct PathBufPy {
    /// The underlying PathBuf instance.
    pathbuf: PathBuf
}

#[pymethods]
impl PathBufPy {
    #[new]
    /// Create a new instance of PathBufPy with an empty PathBuf.
    pub fn new() -> Self {
        PathBufPy {
            pathbuf: PathBuf::new(),
        }
    }

    #[staticmethod]
    /// Create a new instance of PathBufPy with the home directory PathBuf.
    pub fn new_from_home_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_home_dir(),
        }
    }

    #[staticmethod]
    /// Create a new instance of PathBufPy with the documents directory PathBuf.
    pub fn new_from_documents_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_documents_dir(),
        }
    }

    #[staticmethod]
    /// Create a new instance of PathBufPy with the desktop directory PathBuf.
    pub fn new_from_desktop_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_desktop_dir(),
        }
    }

    #[staticmethod]
    /// Create a new instance of PathBufPy with the default Apollo robots directory PathBuf.
    pub fn new_from_default_apollo_robots_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_default_apollo_robots_dir()
        }
    }

    #[staticmethod]
    /// Create a new instance of PathBufPy with the default Apollo environments directory PathBuf.
    pub fn new_from_default_apollo_environments_dir() -> Self {
        Self {
            pathbuf: PathBuf::new_from_default_apollo_environments_dir()
        }
    }

    /// Append a string to the PathBuf and return a new PathBufPy instance.
    pub fn append(&self, s: &str) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append(s),
        }
    }

    /// Append a vector of strings to the PathBuf and return a new PathBufPy instance.
    pub fn append_vec(&self, v: Vec<String>) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append_vec(&v),
        }
    }

    /// Append a string to the PathBuf without a separator and return a new PathBufPy instance.
    pub fn append_without_separator(&self, s: &str) -> Self {
        Self {
            pathbuf: self.pathbuf.clone().append_without_separator(s),
        }
    }

    /// Append another PathBuf to the PathBuf and return a new PathBufPy instance.
    pub fn append_path(&self, s: &Self) -> Self {
        let p = s.pathbuf.clone();
        Self {
            pathbuf: self.pathbuf.clone().append_path(p),
        }
    }

    /// Split the PathBuf into a vector of strings and return it.
    pub fn split_into_strings(&self) -> Vec<String> {
        self.pathbuf.clone().split_into_strings()
    }

    /// Split the PathBuf into a vector of PathBufPy instances.
    pub fn split_into_path_bufs(&self) -> Vec<Self> {
        let mut out = vec![];

        let res = self.pathbuf.clone().split_into_path_bufs();
        res.iter().for_each(|x| {
            out.push(Self {
                pathbuf: x.clone(),
            });
        });

        out
    }

    /// Walk the directory and find the first PathBuf that matches the given criteria.
    pub fn walk_directory_and_find_first(&self, s: &Self) -> Option<Self> {
        let res = self.pathbuf.clone().walk_directory_and_find_first_result(s.pathbuf.clone());
        return match res {
            Ok(res) => { Some(Self { pathbuf: res }) }
            Err(_) => { None }
        }
    }

    /// Walk the directory and find all PathBufs that match the given criteria.
    pub fn walk_directory_and_find_all(&self, s: &Self) -> Vec<Self> {
        let res = self.pathbuf.clone().walk_directory_and_find_all(s.pathbuf.clone());
        let out = res.iter().map(|x| Self { pathbuf: x.clone() }).collect();
        out
    }

    /// Create the directory specified by the PathBuf.
    pub fn create_directory(&self) {
        self.pathbuf.clone().create_directory();
    }

    /// Delete the file specified by the PathBuf.
    pub fn delete_file(&self) {
        self.pathbuf.clone().delete_file();
    }

    /// Delete the directory specified by the PathBuf.
    pub fn delete_directory(&self) {
        self.pathbuf.clone().delete_directory();
    }

    /// Delete all items in the directory specified by the PathBuf.
    pub fn delete_all_items_in_directory(&self) {
        self.pathbuf.clone().delete_all_items_in_directory();
    }

    /// Copy a file to the destination PathBuf.
    pub fn copy_file_to_destination_file_path(&self, destination: &Self) {
        self.pathbuf.clone().copy_file_to_destination_file_path(&destination.pathbuf.clone());
    }

    /// Copy a file to the destination directory specified by the PathBuf.
    pub fn copy_file_to_destination_directory(&self, destination: &Self) {
        self.pathbuf.clone().copy_file_to_destination_directory(&destination.pathbuf)
    }

    /// Extract the last `n` segments of the PathBuf and return as a new PathBufPy instance.
    pub fn extract_last_n_segments(&self, n: usize) -> Self {
        let res = self.pathbuf.clone().extract_last_n_segments(n);
        Self {
            pathbuf: res.clone()
        }
    }

    /// Get all items in the directory specified by the PathBuf, with options to include/exclude directories and files.
    pub fn get_all_items_in_directory(&self, include_directories: bool, include_hidden_directories: bool, include_files: bool, include_hidden_files: bool) -> Vec<Self> {
        let res = self.pathbuf.clone().get_all_items_in_directory(include_directories, include_hidden_directories, include_files, include_hidden_files);
        return res.iter().map(|x| Self { pathbuf: x.clone() }).collect()
    }

    /// Get all filenames in the directory specified by the PathBuf.
    pub fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String> {
        self.pathbuf.clone().get_all_filenames_in_directory(include_hidden_files)
    }

    /// Read the contents of the file specified by the PathBuf into a string, returning an Option.
    pub fn read_file_contents_to_string(&self) -> Option<String> {
        let res = self.pathbuf.clone().read_file_contents_to_string_result();
        match res {
            Ok(res) => { Some(res) }
            Err(_) => { None }
        }
    }

    /// Write a string to the file specified by the PathBuf.
    pub fn write_string_to_file(&self, s: &str) {
        self.pathbuf.clone().write_string_to_file(&s.to_string())
    }

    /// Convert the PathBuf to a string representation.
    pub fn to_string(&self) -> String {
        self.pathbuf.to_str().unwrap().to_string()
    }
}

#[pymodule]
fn apollo_rust_file_pyo3(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // Add the PathBufPy class to the module.
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

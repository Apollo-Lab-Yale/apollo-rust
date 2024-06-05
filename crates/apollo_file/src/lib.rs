use std::fs;
use std::path::{Path, PathBuf};
use walkdir::WalkDir;

pub trait ApolloPathBufTrait: Sized {
    fn new_from_append(s: &str) -> Self;
    fn new_from_home_dir() -> Self;
    fn new_from_documents_dir() -> Self;
    fn new_from_desktop_dir() -> Self;
    fn new_from_walk_dir_and_find<P: AsRef<Path>>(s: P) -> Self;
    fn append(self, s: &str) -> Self;
    fn append_vec(self, v: &Vec<String>) -> Self;
    fn append_path<P: AsRef<Path>>(self, s: P) -> Self;
    fn split_into_strings(&self) -> Vec<String>;
    fn split_into_path_bufs(&self) -> Vec<Self>;
    fn walk_directory_and_find_first<P: AsRef<Path>>(self, s: P) -> Self;
    fn walk_directory_and_find_all<P: AsRef<Path>>(self, s: P) -> Vec<Self>;
    fn create_directory(&self);
    fn delete_file(&self);
    fn delete_directory(&self);
    fn delete_all_items_in_directory(&self);
    fn copy_file_to_destination_file_path(&self, destination: &Self);
    fn copy_file_to_destination_directory(&self, destination: &Self);
    fn extract_last_n_segments(&self, n: usize) -> Self;
    fn get_all_items_in_directory(&self, include_directories: bool, include_files: bool, include_hidden_files: bool) -> Vec<Self>;
    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String>;
}

impl ApolloPathBufTrait for PathBuf {
    fn new_from_append(s: &str) -> Self {
        let mut p = PathBuf::new();
        p.push(std::path::MAIN_SEPARATOR_STR);
        return p.append(s);
    }
    fn new_from_home_dir() -> Self {
        dirs::home_dir().unwrap().to_path_buf()
    }
    fn new_from_documents_dir() -> Self {
        dirs::document_dir().unwrap().to_path_buf()
    }
    fn new_from_desktop_dir() -> Self {
        dirs::desktop_dir().unwrap().to_path_buf()
    }
    fn new_from_walk_dir_and_find<P: AsRef<Path>>(s: P) -> Self {
        let p = PathBuf::new_from_home_dir();
        return p.walk_directory_and_find_first(s);
    }
    fn append(self, s: &str) -> Self {
        let mut out = self.clone();

        let do_s1 = s.find("/").is_some();
        let do_s2 = s.find(r"\").is_some();

        if do_s1 && do_s2 { panic!(r"cannot have both / and \ in append"); }

        if do_s1 {
            let s1 = s.split("/");
            // out.push("/");
            s1.for_each(|x| { out.push(x) });
        } else if do_s2 {
            let s2 = s.split(r"\");
            // out.push(r"\");
            s2.for_each(|x| { out.push(x) });
        } else {
            out.push(s);
        }

        out
    }
    fn append_vec(self, v: &Vec<String>) -> Self {
        let mut out = PathBuf::new();
        for s in v {
            out = out.append(s);
        }
        out
    }
    fn append_path<P: AsRef<Path>>(self, s: P) -> Self {
        let ss = s.as_ref().to_str().expect("error");
        println!(" >>> {:?}", self);
        println!(" >> {:?}", ss);
        return self.append(ss);
    }
    fn split_into_strings(&self) -> Vec<String> {
        let mut out = vec![];
        let s = self.to_str().expect("error");

        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![s.to_string()] };

        let ss = s.split(pat);
        ss.for_each(|x| out.push(x.to_string()));

        out
    }
    fn split_into_path_bufs(&self) -> Vec<Self> {
        let mut out = vec![];
        let s = self.to_str().expect("error");

        let do_s1 = s.contains("/");
        let do_s2 = s.contains(r"\");

        let pat = if do_s1 { "/" } else if do_s2 { r"\" } else { return vec![self.clone()] };

        let ss = s.split(pat);
        ss.for_each(|x| out.push(PathBuf::new_from_append(x)));

        out
    }
    fn walk_directory_and_find_first<P: AsRef<Path>>(self, s: P) -> Self {
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res {
                let entry_path = entry.into_path();
                if entry_path.ends_with(&s) { return entry_path; }
            }
        }

        panic!("could not find");
    }
    fn walk_directory_and_find_all<P: AsRef<Path>>(self, s: P) -> Vec<Self> {
        let mut out = vec![];
        for entry_res in WalkDir::new(self) {
            if let Ok(entry) = entry_res {
                let entry_path = entry.into_path();
                if entry_path.ends_with(&s) { out.push(entry_path); }
            }
        }
        out
    }
    fn create_directory(&self) {
        if self.exists() { return; }
        fs::create_dir_all(self).expect("could not create directory");
    }
    fn delete_file(&self) {
        fs::remove_file(self).expect("could not delete file");
    }
    fn delete_directory(&self) {
        fs::remove_dir_all(self).expect("could not delete directory");
    }
    fn delete_all_items_in_directory(&self) {
        self.delete_directory();
        self.create_directory();
    }
    fn copy_file_to_destination_file_path(&self, destination: &Self) {
        assert!(self.is_file(), "must be a file");
        // assert!(destination.is_file(), "destination must be file path");

        if !destination.exists() {
            let par = destination.parent().expect("error").to_path_buf();
            par.create_directory();
        }
        fs::copy(self, destination).expect("could not copy");
    }
    fn copy_file_to_destination_directory(&self, destination: &Self) {
        assert!(self.is_file(), "must be a file");
        // assert!(destination.is_dir(), "destination must be directory");

        let f = self.file_name().unwrap().to_str().unwrap();
        let d = destination.clone().append(f);

        self.copy_file_to_destination_file_path(&d);
    }
    fn extract_last_n_segments(&self, n: usize) -> Self {
        assert!(n > 0);

        let s = self.split_into_path_bufs();
        assert!(s.len() > n);

        let mut out = PathBuf::new();
        for i in 0..n {
            out = out.append_path( &s[s.len() - (n - i)] );
        }

        out
    }
    fn get_all_items_in_directory(&self, include_directories: bool, include_files: bool, include_hidden_files: bool) -> Vec<Self> {
        let mut out = vec![];

        let res = self.read_dir();
        if let Ok(read_dir) = res {
            for dir_entry_res in read_dir {
                if let Ok(dir_entry) = dir_entry_res {
                    let filename = dir_entry.file_name();
                    let f = filename.to_str().unwrap().to_string();
                    if include_directories && dir_entry.path().is_dir() {
                        out.push(dir_entry.path());
                    }
                    else if include_files && dir_entry.path().is_file() {
                        if include_hidden_files {
                            out.push(dir_entry.path());
                        } else {
                            if !(f.chars().nth(0).unwrap().to_string() == ".") {
                                out.push(dir_entry.path());
                            }
                        }
                    }

                }
            }
        }

        out
    }
    fn get_all_filenames_in_directory(&self, include_hidden_files: bool) -> Vec<String> {
        let mut out = vec![];
        let items = self.get_all_items_in_directory(false, true, include_hidden_files);

        items.iter().for_each(|x| {
           out.push(x.file_name().expect("error").to_str().expect("error").to_string())
        });

        out
    }
}
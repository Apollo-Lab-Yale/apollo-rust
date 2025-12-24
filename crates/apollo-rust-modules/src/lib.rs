use apollo_rust_file::ApolloPathBufTrait;
use std::path::PathBuf;

pub mod robot_modules;

/*
#[derive(Clone, Debug)]
pub struct ResourcesRobotsDirectory {
    pub directory: PathBuf
}

#[derive(Clone, Debug)]
pub struct ResourcesSingleRobotDirectory {
    pub robot_name: String,
    pub robots_directory: PathBuf,
    pub directory: PathBuf
}
*/

#[derive(Clone, Debug)]
pub struct ResourcesRootDirectory {
    pub directory: PathBuf,
    pub resources_type: ResourcesType,
}
impl ResourcesRootDirectory {
    pub fn new(directory: PathBuf, resources_type: ResourcesType) -> Self {
        Self {
            directory,
            resources_type,
        }
    }

    pub fn new_from_default_apollo_robots_dir() -> Self {
        return Self {
            directory: PathBuf::new_from_default_apollo_robots_dir(),
            resources_type: ResourcesType::Robot,
        };
    }

    pub fn new_from_default_apollo_environments_dir() -> Self {
        return Self {
            directory: PathBuf::new_from_default_apollo_environments_dir(),
            resources_type: ResourcesType::Environment,
        };
    }

    pub fn get_subdirectory(&self, name: &str) -> ResourcesSubDirectory {
        let directory = self.directory().clone().append(name);
        assert!(
            directory.exists(),
            "{}",
            format!("directory {:?} does not exist", directory)
        );
        ResourcesSubDirectory::new_raw(
            name.to_string(),
            self.directory().clone(),
            directory,
            self.resources_type,
        )
    }

    pub fn get_subdirectory_option(&self, name: &str) -> Option<ResourcesSubDirectory> {
        let directory = self.directory().clone().append(name);
        return if directory.exists() {
            Some(ResourcesSubDirectory::new_raw(
                name.to_string(),
                self.directory().clone(),
                directory,
                self.resources_type,
            ))
        } else {
            None
        };
    }

    pub fn get_all_subdirectories(&self) -> Vec<ResourcesSubDirectory> {
        let mut out = vec![];

        let items = self
            .directory()
            .get_all_items_in_directory(true, false, false, false);
        items.iter().for_each(|x| {
            let s = x.split_into_strings();
            let name = s.last().unwrap();
            out.push(self.get_subdirectory(name));
        });

        out
    }

    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }

    pub fn new_from_directory(directory: PathBuf, resources_type: ResourcesType) -> Self {
        Self {
            directory,
            resources_type,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ResourcesSubDirectory {
    pub name: String,
    pub root_directory: PathBuf,
    pub directory: PathBuf,
    pub resources_type: ResourcesType,
}
impl ResourcesSubDirectory {
    pub fn new_raw(
        name: String,
        root_directory: PathBuf,
        directory: PathBuf,
        resources_type: ResourcesType,
    ) -> Self {
        Self {
            name,
            root_directory,
            directory,
            resources_type,
        }
    }

    pub fn new_from_path(path: PathBuf, resources_type: ResourcesType) -> Self {
        let directory = path.clone();
        let name = directory.file_name().unwrap().to_str().unwrap().to_string();
        let root_directory = directory.parent().unwrap().to_path_buf();
        Self {
            name,
            root_directory,
            directory,
            resources_type,
        }
    }

    pub fn new_from_path_with_name(
        path: PathBuf,
        name: String,
        resources_type: ResourcesType,
    ) -> Self {
        let directory = path.clone();
        let root_directory = directory.parent().unwrap().to_path_buf();
        Self {
            name,
            root_directory,
            directory,
            resources_type,
        }
    }

    pub fn name(&self) -> &String {
        &self.name
    }

    pub fn root_directory(&self) -> &PathBuf {
        &self.root_directory
    }

    pub fn directory(&self) -> &PathBuf {
        &self.directory
    }

    /// Resolves a relative path to a full path, handling potential naming mismatches
    /// between the internal robot/environment name and the disk directory name.
    pub fn resolve_path(&self, rel_path: &PathBuf) -> PathBuf {
        let root = &self.root_directory;
        let internal_name = &self.name;

        let path_str = rel_path.to_str().expect("error");

        // If the relative path starts with the internal name, we strip it and append to the actual directory.
        if path_str.starts_with(internal_name) {
            let stripped = &path_str[internal_name.len()..];
            let stripped = stripped.trim_start_matches('/');
            let stripped = stripped.trim_start_matches('\\');
            return self.directory.clone().append(stripped);
        }

        // Fallback: append to root directory (original behavior but more robust than hardcoded folder name)
        root.clone().append_path(rel_path)
    }
}

#[derive(Clone, Debug, Copy)]
pub enum ResourcesType {
    Robot,
    Environment,
}

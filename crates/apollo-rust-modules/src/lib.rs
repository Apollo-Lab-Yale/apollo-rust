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
pub struct ResourcesRootDirectory<P = PathBuf> {
    pub directory: P,
    pub resources_type: ResourcesType,
}

pub type NativeResourcesRootDirectory = ResourcesRootDirectory<PathBuf>;

impl<P: ApolloPathBufTrait + Clone> ResourcesRootDirectory<P> {
    pub fn new(directory: P, resources_type: ResourcesType) -> Self {
        Self {
            directory,
            resources_type,
        }
    }

    pub fn get_subdirectory(&self, name: &str) -> ResourcesSubDirectory<P> {
        let directory = self.directory().clone().append(name);
        assert!(
            directory.path_exists(),
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

    pub fn get_subdirectory_option(&self, name: &str) -> Option<ResourcesSubDirectory<P>> {
        let directory = self.directory().clone().append(name);
        return if directory.path_exists() {
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

    pub fn get_all_subdirectories(&self) -> Vec<ResourcesSubDirectory<P>> {
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

    pub fn directory(&self) -> &P {
        &self.directory
    }

    pub fn new_from_directory(directory: P, resources_type: ResourcesType) -> Self {
        Self {
            directory,
            resources_type,
        }
    }
}

impl ResourcesRootDirectory<PathBuf> {
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
}

#[derive(Clone, Debug)]
pub struct ResourcesSubDirectory<P = PathBuf> {
    pub name: String,
    pub root_directory: P,
    pub directory: P,
    pub resources_type: ResourcesType,
}

pub type NativeResourcesSubDirectory = ResourcesSubDirectory<PathBuf>;

impl<P: ApolloPathBufTrait + Clone> ResourcesSubDirectory<P> {
    pub fn new_raw(
        name: String,
        root_directory: P,
        directory: P,
        resources_type: ResourcesType,
    ) -> Self {
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

    pub fn root_directory(&self) -> &P {
        &self.root_directory
    }

    pub fn directory(&self) -> &P {
        &self.directory
    }

    /// Resolves a relative path to a full path, handling potential naming mismatches
    /// between the internal robot/environment name and the disk directory name.
    pub fn resolve_path(&self, rel_path: &P) -> P {
        let root = &self.root_directory;
        let internal_name = &self.name;

        let path_components = rel_path.split_into_strings();
        if let Some(first) = path_components.first() {
            if first == internal_name {
                let mut out = self.directory.clone();
                for segment in path_components.iter().skip(1) {
                    out = out.append(segment);
                }
                return out;
            }
        }

        // Fallback: append to root directory
        root.clone().append_another(rel_path)
    }
}

impl ResourcesSubDirectory<PathBuf> {
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
}

#[derive(Clone, Debug, Copy)]
pub enum ResourcesType {
    Robot,
    Environment,
}

use std::path::PathBuf;
use apollo_rust_file::ApolloPathBufTrait;

#[derive(Clone, Debug)]
pub struct ResourcesEnvironmentsDirectory {
    pub directory: PathBuf
}
impl ResourcesEnvironmentsDirectory {
    pub fn new(directory: PathBuf) -> Self {
        Self { directory }
    }

    pub fn new_default() -> Self {
        Self::new(PathBuf::new_from_default_apollo_environments_dir())
    }

    pub fn get_environment_subdirectory(&self, environment_name: &str) -> ResourcesSingleEnvironmentDirectory {
        let directory = self.directory.clone().append(environment_name);
        assert!(directory.exists(), "{}", format!("directory {:?} does not exist", directory));
        ResourcesSingleEnvironmentDirectory {
            environment_name: environment_name.to_string(),
            environments_directory: self.directory.clone(),
            directory
        }
    }

    pub fn get_all_environment_subdirectories(&self) -> Vec<ResourcesSingleEnvironmentDirectory> {
        let mut out = vec![];

        let items = self.directory.get_all_items_in_directory(true, false, false, false);
        items.iter().for_each(|x| {
            let environment_name = x.iter().last().unwrap().to_str().unwrap().to_string();
            out.push(ResourcesSingleEnvironmentDirectory { environment_name, environments_directory: self.directory.clone(), directory: x.clone() })
        });

        out
    }
}


#[derive(Clone, Debug)]
pub struct ResourcesSingleEnvironmentDirectory {
    pub environment_name: String,
    pub environments_directory: PathBuf,
    pub directory: PathBuf
}

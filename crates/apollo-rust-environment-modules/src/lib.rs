pub mod environment_description_module;
pub mod environment_link_simulation_modes_module;

use std::path::PathBuf;
use serde::{Deserialize, Serialize};

pub struct ResourcesEnvironmentsDirectory {
    pub directory: PathBuf
}

pub struct ResourcesSingleEnvironmentDirectory {
    pub environment_name: String,
    pub environments_directory: PathBuf,
    pub directory: PathBuf
}


#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EnvironmentCreator {

}
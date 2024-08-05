pub mod environment_description_module;
pub mod environment_link_simulation_modes_module;
pub mod mesh_modules;

use std::path::PathBuf;

pub struct ResourcesEnvironmentsDirectory {
    pub directory: PathBuf
}

pub struct ResourcesSingleEnvironmentDirectory {
    pub environment_name: String,
    pub environments_directory: PathBuf,
    pub directory: PathBuf
}

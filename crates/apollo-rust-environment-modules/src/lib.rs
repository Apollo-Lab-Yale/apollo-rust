pub mod environment_description_module;
pub mod mesh_modules;
pub mod environment_dof_module;
pub mod environment_bounds_module;
pub mod environment_connections_module;
pub mod environment_chain_module;

use std::path::PathBuf;

pub struct ResourcesEnvironmentsDirectory {
    pub directory: PathBuf
}

pub struct ResourcesSingleEnvironmentDirectory {
    pub environment_name: String,
    pub environments_directory: PathBuf,
    pub directory: PathBuf
}

use std::path::PathBuf;
use std::process::{Command, Stdio};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_file::traits::FromJsonString;
use apollo_rust_robot_modules::ResourcesSubDirectory;
use apollo_rust_robot_modules::robot_modules::bevy_modules::first_look_vis_module::ApolloFirstLookVisModule;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;


impl PreprocessorModule for ApolloFirstLookVisModule {
    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "bevy_modules/first_look_vis_module".to_string()
    }

    fn current_version() -> String {
        "0.0.1".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        progress_bar.update_with_percentage_preset(0.0);

        let fp = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-preprocessor");
        let output = Command::new("cargo")
            .current_dir(fp.to_str().expect("error"))
            .arg("run")
            .arg("--bin")
            .arg("first_look_vis_module_process")
            .arg("--")
            .arg(s.root_directory.to_str().expect("error"))
            .arg(&s.name)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .expect("Failed to execute cargo command");

        if output.status.success() {
            let stdout = String::from_utf8_lossy(&output.stdout).trim().to_string();
            let split = stdout.split("\n");
            let ss: Vec<String> = split.map(|x| x.to_string()).collect();
            let last = ss.last().expect("error").trim().to_string();

            let res = bool::from_json_string(&last);
            progress_bar.done_preset();
            return if res {
                Ok(Self { done: () })
            } else {
                Err(format!("First look vis module for chain {:?} was not verified.", s.name))
            }
        }

        return Err(format!("First look vis module for chain {:?} could not be built.", s.name))
    }
}
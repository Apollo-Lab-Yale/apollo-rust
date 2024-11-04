use std::path::PathBuf;
use std::process::{Command, Stdio};
use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_modules::ResourcesSubDirectory;
use apollo_rust_modules::robot_modules::link_shapes_modules::link_shapes_skips_module::ApolloLinkShapesSkipsModule;
use crate::PreprocessorModule;
use crate::utils::progress_bar::ProgressBarWrapper;
use apollo_rust_file::traits::FromJsonString;

impl PreprocessorModule for ApolloLinkShapesSkipsModule {
    fn relative_file_path_str_from_sub_dir_to_module_dir() -> String {
        "link_shapes_modules/link_shapes_skips_module".to_string()
    }

    fn current_version() -> String {
        "0.0.2".to_string()
    }

    fn build_raw(s: &ResourcesSubDirectory, progress_bar: &mut ProgressBarWrapper) -> Result<Self, String> {
        progress_bar.update_with_percentage_preset(0.0);

        let fp = PathBuf::new_from_documents_dir().append("apollo-rust/crates/apollo-rust-preprocessor");
        let output = Command::new("cargo")
            .current_dir(fp.to_str().expect("error"))
            .arg("run")
            .arg("--bin")
            .arg("link_shapes_skips_module_process")
            .arg("--")
            .arg(&s.name)
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .output()
            .expect("Failed to execute cargo command");

        if output.status.success() {
            let stdout = String::from_utf8_lossy(&output.stdout).trim().to_string();
            let split = stdout.split("\n");
            let ss: Vec<String> = split.map(|x| x.to_string()).collect();

            let l = ss.len();
            let full_convex_hulls_skips_string = ss[l - 6].trim().to_string();
            let full_obbs_skips_string = ss[l - 5].trim().to_string();
            let full_bounding_spheres_skips_string = ss[l - 4].trim().to_string();
            let decomposition_convex_hulls_skips_string = ss[l - 3].trim().to_string();
            let decomposition_obbs_skips_string = ss[l - 2].trim().to_string();
            let decomposition_bounding_spheres_skips_string = ss[l - 1].trim().to_string();

            let full_convex_hulls_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&full_convex_hulls_skips_string);
            let full_obbs_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&full_obbs_skips_string);
            let full_bounding_spheres_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&full_bounding_spheres_skips_string);
            let decomposition_convex_hulls_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&decomposition_convex_hulls_skips_string);
            let decomposition_obbs_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&decomposition_obbs_skips_string);
            let decomposition_bounding_spheres_skips: Vec<Vec<bool>> = FromJsonString::from_json_string(&decomposition_bounding_spheres_skips_string);

            progress_bar.done_preset();

            return Ok(Self {
                full_convex_hulls_skips,
                full_obbs_skips,
                full_bounding_spheres_skips,
                decomposition_convex_hulls_skips,
                decomposition_obbs_skips,
                decomposition_bounding_spheres_skips,
            });
        }

        return Err(format!("link shapes skips module for chain {:?} could not be built.", s.name))
    }
}
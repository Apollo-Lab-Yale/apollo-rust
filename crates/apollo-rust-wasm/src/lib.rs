use wasm_bindgen::prelude::*;

pub mod kinematics;
pub mod proximity;
pub mod robot_modules;
pub mod web_path_buf;

#[wasm_bindgen(start)]
pub fn start() {
    console_error_panic_hook::set_once();
    web_sys::console::log_1(&"Apollo Rust WASM Initialized".into());
}

#[wasm_bindgen]
pub fn inject_file(path: &str, content: Vec<u8>) {
    web_path_buf::WebPathBuf::insert_file(path, content);
}

#[wasm_bindgen]
pub fn preprocess_standalone_robot(
    urdf_path: &str,
    mesh_dir: &str,
    output_dir: &str,
) -> Result<(), String> {
    use apollo_rust_file::ApolloPathBufTrait;
    use apollo_rust_preprocessor::standalone_preprocessor::StandalonePreprocessor;

    let urdf_pb = web_path_buf::WebPathBuf::new_from_str(urdf_path);
    let mesh_dir_pb = web_path_buf::WebPathBuf::new_from_str(mesh_dir);
    let output_dir_pb = web_path_buf::WebPathBuf::new_from_str(output_dir);

    StandalonePreprocessor::process_robot(&urdf_pb, &mesh_dir_pb, &output_dir_pb)
}

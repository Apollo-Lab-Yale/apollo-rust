use apollo_rust_file::ApolloPathBufTrait;
use apollo_rust_xacro::process_xacro_to_file;
use std::collections::HashMap;
use std::path::PathBuf;

fn main() {
    let xacro_path = PathBuf::new_from_str(
        "/Users/dannyrakita/Desktop/Universal_Robots_ROS2_Description-rolling/urdf/ur.urdf.xacro",
    );
    let dir = PathBuf::new_from_str(
        "/Users/dannyrakita/Desktop/Universal_Robots_ROS2_Description-rolling",
    );
    let output_path = PathBuf::new_from_desktop_dir().append("test.urdf");

    // let mut args = HashMap::new();
    // args.insert("ur_type".to_string(), "ur5".to_string());

    process_xacro_to_file(&xacro_path, &[dir], &output_path, None).expect("error");
}

use apollo_rust_xacro::parser::XacroParser;
use std::path::PathBuf;

fn main() {
    let xacro_content = r#"
<robot name="$(arg robot_name)" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:arg name="robot_name" default="my_robot"/>
    <link name="base_link"/>
</robot>
"#;

    let base_path = PathBuf::from(".");
    let search_dirs = vec![];

    println!("Testing robot tag with $(arg)...");
    match XacroParser::parse(xacro_content, &base_path, &search_dirs, None) {
        Ok(out) => {
            println!("Success!");
            println!("Output: {}", out);
        }
        Err(e) => {
            eprintln!("Caught expected/unexpected error: {:?}", e);
        }
    }

    let xacro_content_2 = r#"
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="p" value="$(arg val)"/>
    <xacro:arg name="val" default="100"/>
    <link name="link_${p}"/>
</robot>
"#;

    println!("\nTesting arg defined after use in property...");
    match XacroParser::parse(xacro_content_2, &base_path, &search_dirs, None) {
        Ok(out) => {
            println!("Success!");
            println!("Output: {}", out);
        }
        Err(e) => {
            eprintln!("Caught expected/unexpected error: {:?}", e);
        }
    }
}

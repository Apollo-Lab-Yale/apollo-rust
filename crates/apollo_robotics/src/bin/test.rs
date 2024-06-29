use std::path::PathBuf;
use apollo_robot_preprocessor::modules::chain_module::ApolloChainModule;
use apollo_robot_preprocessor::{RobotPreprocessorModule, RobotPreprocessorRobotsDirectory};
use apollo_robot_preprocessor::modules::dof_module::ApolloDOFModule;
use apollo_file::ApolloPathBufTrait;
use apollo_linalg::V;
use apollo_robotics::robot_functions::robot_kinematics_functions::RobotKinematics;
use apollo_robotics::urdf_module_nalgebra::ApolloURDFModuleNalgebra;

fn main() {
    let root = RobotPreprocessorRobotsDirectory::new(PathBuf::new_from_documents_dir().append("apollo_robots_dir/robots"));
    let s = root.get_robot_subdirectory("ur5");

    let u = ApolloURDFModuleNalgebra::from_robot_directory(&s);
    let c = ApolloChainModule::load_or_build(&s, false).expect("error");
    let d = ApolloDOFModule::load_or_build(&s, false).expect("error");

    let fk_res = RobotKinematics::fk(&V::from_column_slice(&[0.1; 6]), &u, &c, &d);
    println!("{:?}", fk_res);
}
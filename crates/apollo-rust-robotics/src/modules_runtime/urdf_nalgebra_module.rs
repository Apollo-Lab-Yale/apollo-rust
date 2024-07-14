use apollo_rust_robot_modules::urdf_module::ApolloURDFModule;
use apollo_rust_robot_modules_builders::{RobotPreprocessorModule, RobotPreprocessorSingleRobotDirectory};
use apollo_rust_robotics_core::modules_runtime::urdf_nalgebra_module::{ApolloURDFJointNalgebra, ApolloURDFLinkNalgebra, ApolloURDFNalgebraModule};

pub trait URDFNalgebraModuleBuilders {
    fn from_robot_directory(s: &RobotPreprocessorSingleRobotDirectory) -> Self;
    fn from_urdf_module(urdf_module: &ApolloURDFModule) -> Self;
}
impl URDFNalgebraModuleBuilders for ApolloURDFNalgebraModule {
    fn from_robot_directory(s: &RobotPreprocessorSingleRobotDirectory) -> Self {
        let urdf_module = ApolloURDFModule::load_or_build(s, false).expect("error");
        Self::from_urdf_module(&urdf_module)
    }

    fn from_urdf_module(urdf_module: &ApolloURDFModule) -> Self {
        Self {
            name: urdf_module.name.clone(),
            links: urdf_module.links.iter().map(|x| ApolloURDFLinkNalgebra::from_apollo_urdf_link(x)).collect(),
            joints: urdf_module.joints.iter().map(|x| ApolloURDFJointNalgebra::from_apollo_urdf_joint(x)).collect(),
            materials: urdf_module.materials.clone(),
        }
    }
}
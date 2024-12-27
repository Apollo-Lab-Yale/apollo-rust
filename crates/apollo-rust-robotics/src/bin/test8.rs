use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_modules::robot_modules::urdf_module::{ApolloURDFAxis, ApolloURDFJointLimit, ApolloURDFJointType, ApolloURDFPose};
use apollo_rust_preprocessor::ResourcesSubDirectoryTrait;
use apollo_rust_preprocessor::robot_modules_preprocessor::{AttachmentPoint, CombinedRobot};

fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let s = r.get_subdirectory("d1");

    CombinedRobot::new("go2d1_floating")
        .attach_robot("go2d1", AttachmentPoint::World, ApolloURDFJointType::Floating, ApolloURDFPose::identity(), ApolloURDFAxis::default(), ApolloURDFJointLimit::default(), None, None)
        .create_and_preprocess(&r)

}
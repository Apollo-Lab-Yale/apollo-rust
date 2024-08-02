use apollo_rust_mesh_utils::trimesh::TriMesh;
use apollo_rust_robot_modules::urdf_module::{ApolloURDFJoint, ApolloURDFLink};

pub struct ApolloEnvironmentCreator {
    environment_name: String,
    environment_links: Vec<ApolloURDFLink>,
    environment_joints: Vec<ApolloURDFJoint>,
    environment_trimeshes: Vec<TriMesh>
}
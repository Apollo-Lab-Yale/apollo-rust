use apollo_rust_proximity::double_group_queries::{DoubleGroupQueryMode, pairwise_group_query_contact, pairwise_group_query_distance, pairwise_group_query_intersection};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use nalgebra::DMatrix;
use parry3d_f64::query::Contact;
use crate::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};

pub struct RobotProximityFunctions;
impl RobotProximityFunctions {
    pub fn self_intersect(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> Vec<((usize, usize), bool)> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_intersection(shapes, &poses, shapes, &poses, &DoubleGroupQueryMode::SkipSymmetricalPairs, skips, early_stop, ())
    }

    pub fn self_distance(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> Vec<((usize, usize), f64)> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_distance(shapes, &poses, shapes, &poses, &DoubleGroupQueryMode::SkipSymmetricalPairs, skips, early_stop, ())
    }

    pub fn self_contact(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool, margin: f64) -> Vec<((usize, usize), Option<Contact>)> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_contact(shapes, &poses, shapes, &poses, &DoubleGroupQueryMode::SkipSymmetricalPairs, skips, early_stop, margin)
    }
}
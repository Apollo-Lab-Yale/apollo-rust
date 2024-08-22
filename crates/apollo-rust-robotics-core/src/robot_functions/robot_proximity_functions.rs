use apollo_rust_proximity::double_group_queries::{DoubleGroupProximityQueryMode, DoubleGroupProximityQueryOutput, pairwise_group_query_contact, pairwise_group_query_distance, pairwise_group_query_intersection};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use nalgebra::DMatrix;
use parry3d_f64::query::Contact;
use apollo_rust_proximity::proxima::proxima1::{Proxima1};
use apollo_rust_proximity::proxima::proxima_core::{ProximaBudget, ProximaTrait};
use apollo_rust_proximity::{ProximityLossFunction, ProximitySimpleOutput};
use crate::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};

pub struct RobotProximityFunctions;
impl RobotProximityFunctions {
    pub fn self_intersect(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_intersection(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, skips, early_stop, ())
    }

    pub fn self_distance(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> DoubleGroupProximityQueryOutput<f64> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_distance(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, skips, early_stop, ())
    }

    pub fn self_contact(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_contact(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, skips, early_stop, margin)
    }

    pub fn double_chain_intersect(link_shapes_module_a: &ApolloLinkShapesModule,
                                  link_poses_a: &Vec<ISE3q>,
                                  link_shape_mode_a: LinkShapeMode,
                                  link_shape_rep_a: LinkShapeRep,
                                  link_shapes_module_b: &ApolloLinkShapesModule,
                                  link_poses_b: &Vec<ISE3q>,
                                  link_shape_mode_b: LinkShapeMode,
                                  link_shape_rep_b: LinkShapeRep,
                                  skips: Option<&DMatrix<bool>>,
                                  early_stop: bool,
                                  double_group_proximity_query_mode: &DoubleGroupProximityQueryMode) -> DoubleGroupProximityQueryOutput<bool> {
        let shapes_a = link_shapes_module_a.get_shapes(link_shape_mode_a, link_shape_rep_a);
        let poses_a = link_shapes_module_a.link_poses_to_shape_poses(link_poses_a, link_shape_mode_a);

        let shapes_b = link_shapes_module_b.get_shapes(link_shape_mode_b, link_shape_rep_b);
        let poses_b = link_shapes_module_b.link_poses_to_shape_poses(link_poses_b, link_shape_mode_b);

        pairwise_group_query_intersection(shapes_a, &poses_a, shapes_b, &poses_b, double_group_proximity_query_mode, skips, early_stop, ())
    }

    pub fn double_chain_distance(link_shapes_module_a: &ApolloLinkShapesModule,
                                  link_poses_a: &Vec<ISE3q>,
                                  link_shape_mode_a: LinkShapeMode,
                                  link_shape_rep_a: LinkShapeRep,
                                  link_shapes_module_b: &ApolloLinkShapesModule,
                                  link_poses_b: &Vec<ISE3q>,
                                  link_shape_mode_b: LinkShapeMode,
                                  link_shape_rep_b: LinkShapeRep,
                                  skips: Option<&DMatrix<bool>>,
                                  early_stop: bool,
                                 double_group_proximity_query_mode: &DoubleGroupProximityQueryMode) -> DoubleGroupProximityQueryOutput<f64> {
        let shapes_a = link_shapes_module_a.get_shapes(link_shape_mode_a, link_shape_rep_a);
        let poses_a = link_shapes_module_a.link_poses_to_shape_poses(link_poses_a, link_shape_mode_a);

        let shapes_b = link_shapes_module_b.get_shapes(link_shape_mode_b, link_shape_rep_b);
        let poses_b = link_shapes_module_b.link_poses_to_shape_poses(link_poses_b, link_shape_mode_b);

        pairwise_group_query_distance(shapes_a, &poses_a, shapes_b, &poses_b, double_group_proximity_query_mode, skips, early_stop, ())
    }

    pub fn double_chain_contact(link_shapes_module_a: &ApolloLinkShapesModule,
                                link_poses_a: &Vec<ISE3q>,
                                link_shape_mode_a: LinkShapeMode,
                                link_shape_rep_a: LinkShapeRep,
                                link_shapes_module_b: &ApolloLinkShapesModule,
                                link_poses_b: &Vec<ISE3q>,
                                link_shape_mode_b: LinkShapeMode,
                                link_shape_rep_b: LinkShapeRep,
                                skips: Option<&DMatrix<bool>>,
                                early_stop: bool,
                                margin: f64,
                                double_group_proximity_query_mode: &DoubleGroupProximityQueryMode) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let shapes_a = link_shapes_module_a.get_shapes(link_shape_mode_a, link_shape_rep_a);
        let poses_a = link_shapes_module_a.link_poses_to_shape_poses(link_poses_a, link_shape_mode_a);

        let shapes_b = link_shapes_module_b.get_shapes(link_shape_mode_b, link_shape_rep_b);
        let poses_b = link_shapes_module_b.link_poses_to_shape_poses(link_poses_b, link_shape_mode_b);

        pairwise_group_query_contact(shapes_a, &poses_a, shapes_b, &poses_b, double_group_proximity_query_mode, skips, early_stop, margin)
    }

    pub fn self_intersect_proxima1(proxima: &mut Proxima1, budget: &ProximaBudget, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>) -> ProximitySimpleOutput<bool> {
        let group = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        proxima.proxima_for_intersection(budget, &group, &poses, &group, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, &ProximityLossFunction::Hinge { threshold: 0.0 }, 8.0, skips, false)
    }

    pub fn self_proximity_proxima1(proxima: &mut Proxima1, budget: &ProximaBudget, loss_function: &ProximityLossFunction, p_norm: f64, cutoff_distance: f64, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, average_distances: Option<&DMatrix<f64>>) -> ProximitySimpleOutput<f64> {
        let group = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        proxima.proxima_for_proximity(budget, &group, &poses, &group, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, loss_function, 8.0, cutoff_distance, skips, average_distances, false)
    }
}
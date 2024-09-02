use apollo_rust_proximity::double_group_queries::{DoubleGroupProximityQueryMode, DoubleGroupProximityQueryOutput, pairwise_group_query_contact, pairwise_group_query_distance, pairwise_group_query_intersection};
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use nalgebra::DMatrix;
use parry3d_f64::query::Contact;
use apollo_rust_proximity::proxima::proxima_core::{ProximaBudget, ProximaOutput, ProximaTrait};
use apollo_rust_proximity::{ProximityLossFunction};
use apollo_rust_proximity::bvh::{Bvh, BvhShape};
use crate::modules_runtime::link_shapes_module::{ApolloLinkShapesModule, LinkShapeMode, LinkShapeRep};

pub struct RobotProximityFunctions;
impl RobotProximityFunctions {
    pub fn self_intersect(link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        pairwise_group_query_intersection(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, skips, early_stop, ())
    }

    pub fn self_intersect_bvh<B: BvhShape>(bvh: &mut Bvh<B>, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        bvh.update(&shapes, &poses);
        let pairs = bvh.intersection_filter(&bvh).iter().filter(|x| x.0 <= x.1).map(|x| x.clone()).collect();

        pairwise_group_query_intersection(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SubsetOfPairs(pairs), skips, early_stop, ())
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

    pub fn self_contact_bvh<B: BvhShape>(bvh: &mut Bvh<B>, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, early_stop: bool, margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let shapes = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        bvh.update(&shapes, &poses);
        let pairs = bvh.distance_filter(&bvh, margin).iter().filter(|x| x.0 <= x.1).map(|x| x.clone()).collect();

        pairwise_group_query_contact(shapes, &poses, shapes, &poses, &DoubleGroupProximityQueryMode::SubsetOfPairs(pairs), skips, early_stop, margin)
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

    pub fn double_chain_intersect_bvh<B: BvhShape>(self_bvh: &mut Bvh<B>,
                                                   other_bvh: &mut Bvh<B>,
                                                   link_shapes_module_a: &ApolloLinkShapesModule,
                                                   link_poses_a: &Vec<ISE3q>,
                                                   link_shape_mode_a: LinkShapeMode,
                                                   link_shape_rep_a: LinkShapeRep,
                                                   link_shapes_module_b: &ApolloLinkShapesModule,
                                                   link_poses_b: &Vec<ISE3q>,
                                                   link_shape_mode_b: LinkShapeMode,
                                                   link_shape_rep_b: LinkShapeRep,
                                                   skips: Option<&DMatrix<bool>>,
                                                   early_stop: bool) -> DoubleGroupProximityQueryOutput<bool> {
        let shapes_a = link_shapes_module_a.get_shapes(link_shape_mode_a, link_shape_rep_a);
        let poses_a = link_shapes_module_a.link_poses_to_shape_poses(link_poses_a, link_shape_mode_a);

        let shapes_b = link_shapes_module_b.get_shapes(link_shape_mode_b, link_shape_rep_b);
        let poses_b = link_shapes_module_b.link_poses_to_shape_poses(link_poses_b, link_shape_mode_b);

        self_bvh.update(&shapes_a, &poses_a);
        other_bvh.update(&shapes_b, &poses_b);
        let pairs = self_bvh.intersection_filter(&other_bvh);

        pairwise_group_query_intersection(shapes_a, &poses_a, shapes_b, &poses_b, &DoubleGroupProximityQueryMode::SubsetOfPairs(pairs), skips, early_stop, ())
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

    pub fn double_chain_contact_bvh<B: BvhShape>(self_bvh: &mut Bvh<B>,
                                                 other_bvh: &mut Bvh<B>,
                                                 link_shapes_module_a: &ApolloLinkShapesModule,
                                                 link_poses_a: &Vec<ISE3q>,
                                                 link_shape_mode_a: LinkShapeMode,
                                                 link_shape_rep_a: LinkShapeRep,
                                                 link_shapes_module_b: &ApolloLinkShapesModule,
                                                 link_poses_b: &Vec<ISE3q>,
                                                 link_shape_mode_b: LinkShapeMode,
                                                 link_shape_rep_b: LinkShapeRep,
                                                 skips: Option<&DMatrix<bool>>,
                                                 early_stop: bool,
                                                 margin: f64) -> DoubleGroupProximityQueryOutput<Option<Contact>> {
        let shapes_a = link_shapes_module_a.get_shapes(link_shape_mode_a, link_shape_rep_a);
        let poses_a = link_shapes_module_a.link_poses_to_shape_poses(link_poses_a, link_shape_mode_a);

        let shapes_b = link_shapes_module_b.get_shapes(link_shape_mode_b, link_shape_rep_b);
        let poses_b = link_shapes_module_b.link_poses_to_shape_poses(link_poses_b, link_shape_mode_b);

        self_bvh.update(&shapes_a, &poses_a);
        other_bvh.update(&shapes_b, &poses_b);
        let pairs = self_bvh.distance_filter(&other_bvh, margin);

        pairwise_group_query_contact(shapes_a, &poses_a, shapes_b, &poses_b, &DoubleGroupProximityQueryMode::SubsetOfPairs(pairs), skips, early_stop, margin)

    }

    pub fn self_intersect_proxima<P: ProximaTrait>(proxima: &mut P, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, frozen: bool) -> ProximaOutput<bool> {
        let group = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        proxima.proxima_for_intersection(&group, &poses, &group, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs,  skips, frozen)
    }

    pub fn self_proximity_proxima<P: ProximaTrait>(proxima: &mut P, budget: &ProximaBudget, loss_function: &ProximityLossFunction, p_norm: f64, cutoff_distance: f64, link_shapes_module: &ApolloLinkShapesModule, link_poses: &Vec<ISE3q>, link_shape_mode: LinkShapeMode, link_shape_rep: LinkShapeRep, skips: Option<&DMatrix<bool>>, average_distances: Option<&DMatrix<f64>>, frozen: bool) -> ProximaOutput<f64> {
        let group = link_shapes_module.get_shapes(link_shape_mode, link_shape_rep);
        let poses = link_shapes_module.link_poses_to_shape_poses(link_poses, link_shape_mode);

        proxima.proxima_for_proximity(budget, &group, &poses, &group, &poses, &DoubleGroupProximityQueryMode::SkipSymmetricalPairs, loss_function, p_norm, cutoff_distance, skips, average_distances, frozen)
    }

    pub fn double_chain_intersect_proxima<P: ProximaTrait>(proxima: &mut P,
                                                           self_link_shapes_module: &ApolloLinkShapesModule,
                                                           self_link_poses: &Vec<ISE3q>,
                                                           self_link_shape_mode: LinkShapeMode,
                                                           self_link_shape_rep: LinkShapeRep,
                                                           other_link_shapes_module: &ApolloLinkShapesModule,
                                                           other_link_poses: &Vec<ISE3q>,
                                                           other_link_shape_mode: LinkShapeMode,
                                                           other_link_shape_rep: LinkShapeRep,
                                                           frozen: bool) -> ProximaOutput<bool> {
        let self_group = self_link_shapes_module.get_shapes(self_link_shape_mode, self_link_shape_rep);
        let self_poses = self_link_shapes_module.link_poses_to_shape_poses(self_link_poses, self_link_shape_mode);

        let other_group = other_link_shapes_module.get_shapes(other_link_shape_mode, other_link_shape_rep);
        let other_poses = other_link_shapes_module.link_poses_to_shape_poses(other_link_poses, other_link_shape_mode);

        proxima.proxima_for_intersection(&self_group, &self_poses, &other_group, &other_poses, &DoubleGroupProximityQueryMode::AllPossiblePairs,  None, frozen)
    }

    pub fn double_chain_proximity_proxima<P: ProximaTrait>(proxima: &mut P,
                                                           budget: &ProximaBudget,
                                                           loss_function: &ProximityLossFunction,
                                                           p_norm: f64,
                                                           cutoff_distance: f64,
                                                           self_link_shapes_module: &ApolloLinkShapesModule,
                                                           self_link_poses: &Vec<ISE3q>,
                                                           self_link_shape_mode: LinkShapeMode,
                                                           self_link_shape_rep: LinkShapeRep,
                                                           other_link_shapes_module: &ApolloLinkShapesModule,
                                                           other_link_poses: &Vec<ISE3q>,
                                                           other_link_shape_mode: LinkShapeMode,
                                                           other_link_shape_rep: LinkShapeRep,
                                                           frozen: bool) -> ProximaOutput<f64> {
        let self_group = self_link_shapes_module.get_shapes(self_link_shape_mode, self_link_shape_rep);
        let self_poses = self_link_shapes_module.link_poses_to_shape_poses(self_link_poses, self_link_shape_mode);

        let other_group = other_link_shapes_module.get_shapes(other_link_shape_mode, other_link_shape_rep);
        let other_poses = other_link_shapes_module.link_poses_to_shape_poses(other_link_poses, other_link_shape_mode);

        proxima.proxima_for_proximity(budget, self_group, &self_poses, other_group, &other_poses, &DoubleGroupProximityQueryMode::AllPossiblePairs, loss_function, p_norm, cutoff_distance, None, None, frozen)
    }
}
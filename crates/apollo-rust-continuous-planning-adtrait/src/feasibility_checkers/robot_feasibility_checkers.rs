use std::sync::{Arc, RwLock};
use apollo_rust_linalg::V;
use apollo_rust_proximity_parry::bvh::{Bvh, BvhShape};
use apollo_rust_proximity_parry::ToIntersectionResult;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_robotics_core::modules_runtime::link_shapes_module::{LinkShapeMode, LinkShapeRep};
use crate::feasibility_checkers::FeasibilityCheckerTrait;

#[derive(Clone)]
pub struct RobotBoundsFeasibilityChecker {
    pub robot_chain: Arc<ChainNalgebra>
}
impl RobotBoundsFeasibilityChecker {
    pub fn new(robot_chain: Arc<ChainNalgebra>) -> Self {
        Self { robot_chain }
    }
}
impl FeasibilityCheckerTrait for RobotBoundsFeasibilityChecker {

    #[inline(always)]
    fn is_feasible_state(&self, state: &V) -> bool {
        let bounds = &self.robot_chain.bounds_module.bounds;
        assert_eq!(bounds.len(), state.len());

        for (b, x) in bounds.iter().zip(state.iter()) {
            if *x < b.0 || *x > b.1 { return false; }
        }

        return true;
    }

}

#[derive(Clone)]
pub struct RobotNaiveFeasibilityChecker {
    bounds_checker: RobotBoundsFeasibilityChecker,
    robot_chain: Arc<ChainNalgebra>,
    environment_chain: Option<Arc<ChainNalgebra>>,
    environment_chain_state: V,
    self_link_shape_mode: LinkShapeMode,
    self_link_shape_rep: LinkShapeRep,
    environment_link_shape_mode: Option<LinkShapeMode>,
    environment_link_shape_rep: Option<LinkShapeRep>
}
impl RobotNaiveFeasibilityChecker {
    pub fn new(robot_chain: Arc<ChainNalgebra>, self_link_shape_mode: LinkShapeMode, self_link_shape_rep: LinkShapeRep) -> Self {
        Self {
            bounds_checker: RobotBoundsFeasibilityChecker::new(robot_chain.clone()),
            robot_chain,
            environment_chain: None,
            environment_chain_state: Default::default(),
            self_link_shape_mode,
            self_link_shape_rep,
            environment_link_shape_mode: None,
            environment_link_shape_rep: None,
        }
    }

    pub fn new_with_environment_chain(robot_chain: Arc<ChainNalgebra>,
                                      self_link_shape_mode: LinkShapeMode,
                                      self_link_shape_rep: LinkShapeRep,
                                      environment_chain: Arc<ChainNalgebra>,
                                      environment_link_shape_mode: LinkShapeMode,
                                      environment_link_shape_rep: LinkShapeRep,
                                      environment_state: &V) -> Self {
        Self {
            bounds_checker: RobotBoundsFeasibilityChecker::new(robot_chain.clone()),
            robot_chain,
            environment_chain: Some(environment_chain),
            environment_chain_state: environment_state.clone(),
            self_link_shape_mode,
            self_link_shape_rep,
            environment_link_shape_mode: Some(environment_link_shape_mode),
            environment_link_shape_rep: Some(environment_link_shape_rep),
        }
    }
}
impl FeasibilityCheckerTrait for RobotNaiveFeasibilityChecker {
    fn is_feasible_state(&self, state: &V) -> bool {
        if !self.bounds_checker.is_feasible_state(state) { return false; }

        if let Some(environment_chain) = &self.environment_chain {
            let res = self.robot_chain.double_chain_intersect_from_states(environment_chain, state, self.self_link_shape_mode, self.self_link_shape_rep, &self.environment_chain_state, self.environment_link_shape_mode.unwrap(), self.environment_link_shape_rep.unwrap(), true).to_intersection_result();
            if res { return false; }
        }

        let res = self.robot_chain.self_intersect_from_state(state, self.self_link_shape_mode, self.self_link_shape_rep, true).to_intersection_result();
        if res { return false; }

        return true;
    }
}

#[derive(Clone)]
pub struct RobotBVHFeasibilityChecker<B: BvhShape> {
    bounds_checker: RobotBoundsFeasibilityChecker,
    robot_chain: Arc<ChainNalgebra>,
    environment_chain: Option<Arc<ChainNalgebra>>,
    environment_chain_state: V,
    self_link_shape_mode: LinkShapeMode,
    self_link_shape_rep: LinkShapeRep,
    environment_link_shape_mode: Option<LinkShapeMode>,
    environment_link_shape_rep: Option<LinkShapeRep>,
    robot_bvh: Arc<RwLock<Bvh<B>>>,
    environment_bvh: Option<Arc<RwLock<Bvh<B>>>>
}
impl<B: BvhShape> RobotBVHFeasibilityChecker<B> {
    pub fn new(robot_chain: Arc<ChainNalgebra>, self_link_shape_mode: LinkShapeMode, self_link_shape_rep: LinkShapeRep, branch_factor: usize) -> Self {
        let robot_bvh = robot_chain.get_bvh::<B>(&robot_chain.zeros_state(), self_link_shape_mode, self_link_shape_rep, branch_factor);
        Self {
            bounds_checker: RobotBoundsFeasibilityChecker::new(robot_chain.clone()),
            robot_chain,
            environment_chain: None,
            environment_chain_state: Default::default(),
            self_link_shape_mode,
            self_link_shape_rep,
            environment_link_shape_mode: None,
            environment_link_shape_rep: None,
            robot_bvh: Arc::new(RwLock::new(robot_bvh)),
            environment_bvh: None,
        }
    }

    pub fn new_with_environment_chain(robot_chain: Arc<ChainNalgebra>,
                                      self_link_shape_mode: LinkShapeMode,
                                      self_link_shape_rep: LinkShapeRep,
                                      environment_chain: Arc<ChainNalgebra>,
                                      environment_link_shape_mode: LinkShapeMode,
                                      environment_link_shape_rep: LinkShapeRep,
                                      environment_state: &V,
                                      branch_factor: usize) -> Self {
        let robot_bvh = robot_chain.get_bvh::<B>(&robot_chain.zeros_state(), self_link_shape_mode, self_link_shape_rep, branch_factor);
        let environment_bvh = environment_chain.get_bvh::<B>(environment_state, environment_link_shape_mode, environment_link_shape_rep, branch_factor);

        Self {
            bounds_checker: RobotBoundsFeasibilityChecker::new(robot_chain.clone()),
            robot_chain,
            environment_chain: Some(environment_chain),
            environment_chain_state: environment_state.clone(),
            self_link_shape_mode,
            self_link_shape_rep,
            environment_link_shape_mode: Some(environment_link_shape_mode),
            environment_link_shape_rep: Some(environment_link_shape_rep),
            robot_bvh: Arc::new(RwLock::new(robot_bvh)),
            environment_bvh: Some(Arc::new(RwLock::new(environment_bvh))),
        }
    }
}
impl<B: BvhShape> FeasibilityCheckerTrait for RobotBVHFeasibilityChecker<B> {
    fn is_feasible_state(&self, state: &V) -> bool {
        if !self.bounds_checker.is_feasible_state(state) { return false; }

        let frames = self.robot_chain.fk(state);
        let mut robot_bvh = self.robot_bvh.write().unwrap();

        if let Some(environment_chain) = &self.environment_chain {
            let environment_frames = environment_chain.fk(&self.environment_chain_state);
            let mut environment_bvh = self.environment_bvh.as_ref().unwrap().write().unwrap();
            let res = self.robot_chain.double_chain_intersect_bvh(&mut robot_bvh, &mut environment_bvh, environment_chain, &frames, self.self_link_shape_mode, self.self_link_shape_rep, &environment_frames, self.environment_link_shape_mode.unwrap(), self.environment_link_shape_rep.unwrap(), true).to_intersection_result();
            if res { return false; }
        }

        let res = self.robot_chain.self_intersect_bvh(&mut robot_bvh, &frames, self.self_link_shape_mode, self.self_link_shape_rep, true).to_intersection_result();
        if res { return false; }

        return true;
    }
}
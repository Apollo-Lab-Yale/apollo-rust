use std::sync::{Arc, Mutex};
use bevy::app::App;
use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_interpolation::InterpolatorTrait;
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_optimization::IterativeOptimizerTrait;
use apollo_rust_optimization::line_searches::backtracking_line_search::BacktrackingLineSearch;
use apollo_rust_optimization::optimizers::bfgs::{BFGS, LBFGS};
use apollo_rust_optimization::optimizers::open::OpENUnconstrained;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::h1::ApolloUnitQuaternionH1LieTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;

pub struct IKObjectiveFunction1 {
    pub c: Arc<ChainNalgebra>,
    pub goal_link_idx: usize,
    pub ik_goal: ISE3q
}
impl FunctionNalgebraTrait for IKObjectiveFunction1 {
    fn call_raw(&self, x: &V) -> V {
        let fk_res = self.c.fk(x);
        let link_frame = &fk_res[self.goal_link_idx];
        let diff = &link_frame.0.translation.vector - &self.ik_goal.0.translation.vector;

        V::new(&[diff.norm().powi(2)])
    }

    fn input_dim(&self) -> usize {
        self.c.num_dofs()
    }

    fn output_dim(&self) -> usize {
        1
    }
}

pub struct IKObjectiveFunction2 {
    pub c: Arc<ChainNalgebra>,
    pub goal_link_idx: usize,
    pub ik_goal: ISE3q
}
impl FunctionNalgebraTrait for IKObjectiveFunction2 {
    fn call_raw(&self, x: &V) -> V {
        let fk_res = self.c.fk(x);
        let link_frame = &fk_res[self.goal_link_idx];
        let position_diff = &link_frame.0.translation.vector - &self.ik_goal.0.translation.vector;
        let orientation_diff = link_frame.0.rotation.to_lie_group_h1().displacement(&self.ik_goal.0.rotation.to_lie_group_h1()).ln().vee();
        // let binding = self.ik_goal.displacement(link_frame).ln();
        // let diff = binding.v();

        let res = position_diff.norm().powi(2) + orientation_diff.norm().powi(2);
        V::new(&[res])
    }

    fn input_dim(&self) -> usize {
        self.c.num_dofs()
    }

    fn output_dim(&self) -> usize {
        1
    }
}


fn main() {
    let r = ResourcesRootDirectory::new_from_default_apollo_robots_dir();
    let c = r.get_subdirectory("ur5").to_chain_nalgebra();
    let ac = c.clone().to_arc_chain();
    let init_condition = V::new(&[0.001; 6]);

    let ee_targets = vec![
        ISE3q::new(I3::from_slices_euler_angles(&[0.2, 0.0, 0.5], &[0.,0.,0.])),
        ISE3q::new(I3::from_slices_euler_angles(&[0.3, 0.0, 0.5], &[0.,0.,0.])),
        ISE3q::new(I3::from_slices_euler_angles(&[0.4, 0.1, 0.6], &[0.,0.,0.])),
        ISE3q::new(I3::from_slices_euler_angles(&[0.5, 0.2, 0.7], &[0.,0.,0.])),
    ];

    // let o = OpENUnconstrained::new(ac.num_dofs(), ac.bounds_module.dof_lower_bounds.clone(), ac.bounds_module.dof_upper_bounds.clone());
    let o = BFGS::new(Arc::new(BacktrackingLineSearch::default()), None);

    let f = Arc::new(Mutex::new(IKObjectiveFunction2 {
        c: ac.clone(),
        goal_link_idx: 9,
        ik_goal: ee_targets[0].clone(),
    }));

    let function_engine = FunctionEngine::new(f.clone(), DerivativeMethodFD::new(0.00001));

    let mut solutions = vec![init_condition.clone()];
    for ee_target in &ee_targets {
        let mut m = f.lock().unwrap();
        m.ik_goal = ee_target.clone();
        drop(m);
        let output = o.optimize_unconstrained(1000, &init_condition, &function_engine);
        solutions.push(output.x_star.clone());
    }

    let spline = InterpolatingSpline::new(solutions, InterpolatingSplineType::Linear)
        .to_timed_interpolator(5.0);

    //////////// drawing code

    let mut app = App::new()
        .apollo_bevy_robotics_base(true)
        .apollo_bevy_spawn_chain_default(&c, 0, ISE3q::identity())
        .apollo_bevy_chain_display(&c);

    for ee_target in &ee_targets {
        app = app.apollo_bevy_draw_frame(&ee_target, 0.1);
    }

    app = app.apollo_bevy_chain_motion_playback(0, spline);

    app.run();
}
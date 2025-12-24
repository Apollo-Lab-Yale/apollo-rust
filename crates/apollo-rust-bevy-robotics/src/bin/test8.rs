use apollo_rust_bevy::ApolloBevyTrait;
use apollo_rust_differentiation::derivative_methods::DerivativeMethodFD;
use apollo_rust_differentiation::{FunctionEngine, FunctionNalgebraTrait};
use apollo_rust_interpolation::splines::{InterpolatingSpline, InterpolatingSplineType};
use apollo_rust_interpolation::InterpolatorTrait;
use apollo_rust_lie::{LieAlgebraElement, LieGroupElement};
use apollo_rust_linalg::{ApolloDVectorTrait, V};
use apollo_rust_modules::ResourcesRootDirectory;
use apollo_rust_robotics::ToChainNalgebra;
use apollo_rust_robotics_core::ChainNalgebra;
use apollo_rust_spatial::isometry3::{ApolloIsometry3Trait, I3};
use apollo_rust_spatial::lie::h1::ApolloUnitQuaternionH1LieTrait;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use bevy::app::App;
use std::sync::Arc;

pub struct IKObjectiveFunction1 {
    pub c: Arc<ChainNalgebra>,
    pub goal_link_idx: usize,
    pub ik_goal: ISE3q,
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
    pub ik_goal: ISE3q,
}
impl FunctionNalgebraTrait for IKObjectiveFunction2 {
    fn call_raw(&self, x: &V) -> V {
        let fk_res = self.c.fk(x);
        let link_frame = &fk_res[self.goal_link_idx];
        let position_diff = &link_frame.0.translation.vector - &self.ik_goal.0.translation.vector;
        let orientation_diff = link_frame
            .0
            .rotation
            .to_lie_group_h1()
            .displacement(&self.ik_goal.0.rotation.to_lie_group_h1())
            .ln()
            .vee();
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

    let ik_goal = ISE3q::new(I3::from_slices_euler_angles(
        &[0.2, 0.0, 0.5],
        &[0., 0., 0.],
    ));

    let objective_function = IKObjectiveFunction2 {
        c: ac.clone(),
        goal_link_idx: 9,
        ik_goal: ik_goal.clone(),
    };

    let function_engine =
        FunctionEngine::new(objective_function, DerivativeMethodFD::new(0.000001));

    let mut output_states = vec![];
    let mut curr_state = V::new(&[0.2; 6]);
    output_states.push(curr_state.clone());
    let lambda = 0.1;
    let max_iters = 2000;

    for _ in 0..max_iters {
        let (f, g) = function_engine.derivative(&curr_state);
        curr_state += -lambda * &g.transpose();
        output_states.push(curr_state.clone());
        println!("{:?}", f[0]);
    }

    let linear_spline = InterpolatingSpline::new(output_states, InterpolatingSplineType::Linear)
        .to_timed_interpolator(5.0);

    let mut app = App::new()
        .apollo_bevy_robotics_base(true, false)
        .apollo_bevy_spawn_chain_default(&c, 0, ISE3q::identity())
        .apollo_bevy_chain_display(&c)
        .apollo_bevy_draw_frame(&ik_goal, 0.1)
        .apollo_bevy_chain_motion_playback(0, linear_spline);

    app.run();
}

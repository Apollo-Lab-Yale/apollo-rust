use ad_trait::AD;
use rand::Rng;
use serde::{Deserialize, Serialize};
use apollo_rust_linalg_adtrait::{ApolloDVectorTrait, V};
use apollo_rust_modules::robot_modules::bounds_module::ApolloBoundsModule;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ApolloBoundsModuleADTrait<A: AD> {
    #[serde(deserialize_with = "Vec::<(A, A)>::deserialize")]
    pub bounds: Vec<(A, A)>,
    #[serde(deserialize_with = "Vec::<A>::deserialize")]
    pub dof_lower_bounds: Vec<A>,
    #[serde(deserialize_with = "Vec::<A>::deserialize")]
    pub dof_upper_bounds: Vec<A>
}
impl<A: AD> ApolloBoundsModuleADTrait<A> {
    pub fn from_apollo_bounds_module(apollo_bounds_module: &ApolloBoundsModule) -> Self {
        Self {
            bounds: apollo_bounds_module.bounds.iter().map(|(x, y)| (x.to_other_ad_type::<A>(), y.to_other_ad_type::<A>())).collect(),
            dof_lower_bounds: apollo_bounds_module.dof_lower_bounds.iter().map(|x| x.to_other_ad_type::<A>()).collect(),
            dof_upper_bounds: apollo_bounds_module.dof_upper_bounds.iter().map(|x| x.to_other_ad_type::<A>()).collect(),
        }
    }

    pub fn to_apollo_bounds_module(&self) -> ApolloBoundsModule {
        let tmp = self.to_other_ad_type::<f64>();
        ApolloBoundsModule {
            bounds: tmp.bounds,
            dof_lower_bounds: tmp.dof_lower_bounds,
            dof_upper_bounds: tmp.dof_upper_bounds,
        }
    }

    pub fn to_other_ad_type<A2: AD>(&self) -> ApolloBoundsModuleADTrait<A2> {
        ApolloBoundsModuleADTrait {
            bounds: self.bounds.iter().map(|(x, y)| (x.to_other_ad_type::<A2>(), y.to_other_ad_type::<A2>())).collect(),
            dof_lower_bounds: self.dof_lower_bounds.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
            dof_upper_bounds: self.dof_upper_bounds.iter().map(|x| x.to_other_ad_type::<A2>()).collect(),
        }
    }

    pub fn sample_random_state(&self) -> V<A> {
        let mut rng = rand::thread_rng();
        let v = V::new(&self.bounds.iter().map(|&(lower, upper)| rng.gen_range(lower.to_constant()..upper.to_constant())).collect::<Vec<f64>>());
        return v.to_other_ad_type::<A>();
    }
}
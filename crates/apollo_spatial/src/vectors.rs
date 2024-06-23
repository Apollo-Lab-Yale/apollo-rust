use nalgebra::{Vector2, Vector3};
use rand::Rng;
use apollo_linalg::{ApolloDMatrixTrait, ApolloDVectorTrait, M, V};

pub type V2 = Vector2<f64>;
pub type V3 = Vector3<f64>;

pub trait ApolloVector3Trait {
    fn from_v(vector: &V) -> Self;
    fn to_v(&self) -> V;
    fn to_m(&self) -> M;
    fn new_random_with_range(min: f64, max: f64) -> Self;
}
impl ApolloVector3Trait for V3 {
    fn from_v(vector: &V) -> Self {
        Self::from_column_slice(vector.as_slice())
    }
    #[inline(always)]
    fn to_v(&self) -> V {
        V::new(self.as_slice())
    }
    #[inline(always)]
    fn to_m(&self) -> M { M::new(self.as_slice(), 3, 1) }
    fn new_random_with_range(min: f64, max: f64) -> Self {
        let mut rng = rand::thread_rng();
        let mut v = V3::zeros();

        for i in 0..3 {
            v[i] = rng.gen_range(min..=max);
        }

        v
    }
}
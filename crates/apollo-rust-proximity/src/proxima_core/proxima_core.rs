

#[derive(Clone, Debug)]
pub struct ProximaOutput {
    pub shape_indices: (usize, usize),
    pub approximate_raw_distance: f64,
    pub lower_bound_raw_distance: f64,
    pub upper_bound_raw_distance: f64
}
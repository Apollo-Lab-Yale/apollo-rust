pub mod splines;

use std::marker::PhantomData;
use nalgebra::DVector;

pub trait InterpolatorTraitLite : Send + Sync {
    fn interpolate(&self, t: f64) -> DVector<f64>;
    fn max_t(&self) -> f64;
    fn interpolate_on_range(&self, range_start: f64, range_stop: f64, t: f64) -> DVector<f64> {
        assert!(range_start <= t && t <= range_stop);
        let ratio = (t - range_start) / (range_stop - range_start);
        return self.interpolate_normalized(ratio);
    }
    fn interpolate_normalized(&self, u: f64) -> DVector<f64> {
        assert!(0.0 <= u && u <= 1.0);

        let ratio = self.max_t() * u;
        return self.interpolate(ratio);
    }
    fn interpolate_points_by_num_points(&self, num_points: usize) -> Vec<DVector<f64>> {
        let mut out = vec![];

        let ts = get_interpolation_range_num_steps(0.0, 1.0, num_points);
        for t in ts { out.push(self.interpolate_normalized(t)); }

        out
    }
    fn interpolate_points_by_normalized_stride(&self, stride_length: f64) -> Vec<DVector<f64>> {
        let mut out = vec![];

        let ts = get_interpolation_range(0.0, 1.0, stride_length);
        for t in ts { out.push(self.interpolate_normalized(t)); }

        out
    }
}
impl<U: InterpolatorTraitLite> InterpolatorTraitLite for Box<U> {
    fn interpolate(&self, t: f64) -> DVector<f64> {
        self.as_ref().interpolate(t)
    }

    #[inline(always)]
    fn max_t(&self) -> f64 {
        self.as_ref().max_t()
    }
}

pub trait InterpolatorTrait: Clone + InterpolatorTraitLite {
    fn to_arclength_parameterized_interpolator(&self, num_arclength_markers: usize) -> ArclengthParameterizedInterpolator<Self> {
        ArclengthParameterizedInterpolator::new(self.clone(), num_arclength_markers)
    }
    fn to_timed_interpolator(&self, max_time: f64) -> TimedInterpolator<Self> {
        TimedInterpolator::new(self.clone(), max_time)
    }
}
impl<U: InterpolatorTraitLite + Clone> InterpolatorTrait for U {}

#[derive(Clone)]
pub struct ArclengthParameterizedInterpolator<I: InterpolatorTrait> {
    interpolator: I,
    arclength_markers: Vec<(f64, f64)>,
    total_arclength: f64,
    phantom_data: PhantomData<DVector<f64>>,
}
impl<I: InterpolatorTrait> ArclengthParameterizedInterpolator<I> {
    pub fn new(interpolator: I, num_arclength_markers: usize) -> Self {
        assert!(num_arclength_markers > 10);

        let mut arclength_markers = vec![];

        let mut t = 0.0;
        let max_allowable_t_value = interpolator.max_t();
        let step_size = max_allowable_t_value / num_arclength_markers as f64;
        let mut accumulated_distance = 0.0;

        let mut prev_point = interpolator.interpolate(0.0);

        let mut passed_m = false;
        while !passed_m {
            if t >= max_allowable_t_value {
                passed_m = true;
                t = max_allowable_t_value;
            }
            let curr_point = interpolator.interpolate(t);
            let dis = (&curr_point - &prev_point).norm();
            accumulated_distance += dis;

            arclength_markers.push((accumulated_distance, t));

            prev_point = curr_point.clone();
            t += step_size;
        }

        Self { interpolator, arclength_markers, total_arclength: accumulated_distance, phantom_data: PhantomData::default() }
    }

    pub fn interpolate_points_by_arclength_absolute_stride(&self, arclength_stride: f64) -> Vec<DVector<f64>> {
        let normalized_stride = arclength_stride / self.total_arclength;
        self.interpolate_points_by_normalized_stride(normalized_stride)
    }
}
impl<I: InterpolatorTrait> InterpolatorTraitLite for ArclengthParameterizedInterpolator<I> {
    fn interpolate(&self, s: f64) -> DVector<f64> {
        assert!(0.0 <= s && s <= 1.0);

        let r = s * self.total_arclength;

        let binary_search_res = self.arclength_markers.binary_search_by(|x| x.0.partial_cmp(&r).unwrap());

        return match binary_search_res {
            Ok(idx) => {
                self.interpolator.interpolate(self.arclength_markers[idx].1)
            }
            Err(idx) => {
                if idx == 0 { return self.interpolator.interpolate(0.0) }
                let upper_bound_idx = idx;
                let lower_bound_idx = idx - 1;

                let upper_bound_dis = self.arclength_markers[upper_bound_idx].0;
                let lower_bound_dis = self.arclength_markers[lower_bound_idx].0;

                assert!(lower_bound_dis <= r && r <= upper_bound_dis);

                let upper_bound_t = self.arclength_markers[upper_bound_idx].1;
                let lower_bound_t = self.arclength_markers[lower_bound_idx].1;

                let dis_ratio = (r - lower_bound_dis) / (upper_bound_dis - lower_bound_dis);

                let t = lower_bound_t + dis_ratio * (upper_bound_t - lower_bound_t);

                self.interpolator.interpolate(t)
            }
        }
    }

    fn max_t(&self) -> f64 {
        1.0
    }
}

#[derive(Clone)]
pub struct TimedInterpolator<I: InterpolatorTrait> {
    interpolator: I,
    max_time: f64,
    phantom_data: PhantomData<DVector<f64>>,
}
impl<I: InterpolatorTrait> TimedInterpolator<I> {
    pub fn new(interpolator: I, max_time: f64) -> Self {
        Self { interpolator, max_time, phantom_data: PhantomData::default() }
    }

    pub fn interpolate_points_by_time_stride(&self, time_stride: f64) -> Vec<DVector<f64>> {
        let mut out = vec![];

        let ts = get_interpolation_range(0.0, self.max_time, time_stride);
        for t in ts { out.push(self.interpolate_normalized(t)); }

        out
    }
}
impl<I: InterpolatorTrait> InterpolatorTraitLite for TimedInterpolator<I> {
    fn interpolate(&self, t: f64) -> DVector<f64> {
        self.interpolator.interpolate_on_range(0.0, self.max_time, t)
    }

    fn max_t(&self) -> f64 {
        self.max_time
    }
}

pub fn get_interpolation_range(range_start: f64, range_stop: f64, step_size: f64) -> Vec<f64> {
    assert!(range_stop >= range_start);

    let mut out_range = Vec::new();
    let mut curr_val = range_start;

    'l: loop {
        out_range.push(curr_val);
        curr_val += step_size;
        if curr_val > range_stop { break 'l; }
    }

    let diff = (range_stop - *out_range.last().unwrap()).abs();
    if diff > 0.001 { out_range.push(range_stop) }

    out_range
}

pub fn get_interpolation_range_num_steps(range_start: f64, range_stop: f64, num_steps: usize) -> Vec<f64> {
    let step_size = (range_stop - range_start) / (num_steps as f64 - 1.0);
    get_interpolation_range(range_start, range_stop, step_size)
}

pub fn linearly_interpolate_points(start_point: DVector<f64>, end_point: DVector<f64>, num_points: usize) -> Vec<DVector<f64>> {
    let mut out = vec![];
    let range = get_interpolation_range_num_steps(0.0, 1.0, num_points);
    for t in &range {
        let p = &start_point * (1.0 - *t) + *t * &end_point;
        out.push(p);
    }
    out
}

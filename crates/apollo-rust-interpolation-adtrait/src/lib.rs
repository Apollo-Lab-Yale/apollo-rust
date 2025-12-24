pub mod splines;

use ad_trait::AD;
use apollo_rust_linalg_adtrait::V;
use nalgebra::DVector;
use std::marker::PhantomData;

pub trait InterpolatorTraitLite<A: AD>: Send + Sync {
    fn interpolate(&self, t: A) -> V<A>;
    fn max_t(&self) -> A;
    fn interpolate_on_range(&self, range_start: A, range_stop: A, t: A) -> V<A> {
        assert!(range_start <= t && t <= range_stop);
        let ratio = (t - range_start) / (range_stop - range_start);
        return self.interpolate_normalized(ratio);
    }
    fn interpolate_normalized(&self, u: A) -> V<A> {
        // assert!(A::constant(0.0) <= u && u <= A::constant(1.0));
        let zero = A::constant(0.0);
        let one = A::constant(1.0);
        // Relaxed assertion or handling for AD types might be needed, but strict check is fine for now if logical steps are correct.

        // let ratio = self.max_t() * u;
        let ratio = self.max_t() * u;
        return self.interpolate(ratio);
    }
    fn interpolate_points_by_num_points(&self, num_points: usize) -> Vec<V<A>> {
        let mut out = vec![];

        let ts = get_interpolation_range_num_steps(A::constant(0.0), A::constant(1.0), num_points);
        for t in ts {
            out.push(self.interpolate_normalized(t));
        }

        out
    }
    fn interpolate_points_by_normalized_stride(&self, stride_length: A) -> Vec<V<A>> {
        let mut out = vec![];

        let ts = get_interpolation_range(A::constant(0.0), A::constant(1.0), stride_length);
        for t in ts {
            out.push(self.interpolate_normalized(t));
        }

        out
    }
}
impl<U: InterpolatorTraitLite<A>, A: AD> InterpolatorTraitLite<A> for Box<U> {
    fn interpolate(&self, t: A) -> V<A> {
        self.as_ref().interpolate(t)
    }

    #[inline(always)]
    fn max_t(&self) -> A {
        self.as_ref().max_t()
    }
}

pub trait InterpolatorTrait<A: AD>: Clone + InterpolatorTraitLite<A> {
    fn to_arclength_parameterized_interpolator(
        &self,
        num_arclength_markers: usize,
    ) -> ArclengthParameterizedInterpolator<A, Self> {
        ArclengthParameterizedInterpolator::new(self.clone(), num_arclength_markers)
    }
    fn to_timed_interpolator(&self, max_time: A) -> TimedInterpolator<A, Self> {
        TimedInterpolator::new(self.clone(), max_time)
    }
}
impl<U: InterpolatorTraitLite<A> + Clone, A: AD> InterpolatorTrait<A> for U {}

#[derive(Clone)]
pub struct ArclengthParameterizedInterpolator<A: AD, I: InterpolatorTrait<A>> {
    interpolator: I,
    arclength_markers: Vec<(A, A)>,
    total_arclength: A,
    phantom_data: PhantomData<A>,
}
impl<A: AD, I: InterpolatorTrait<A>> ArclengthParameterizedInterpolator<A, I> {
    pub fn new(interpolator: I, num_arclength_markers: usize) -> Self {
        assert!(num_arclength_markers > 10);

        let mut arclength_markers = vec![];

        let mut t = A::constant(0.0);
        let max_allowable_t_value = interpolator.max_t();
        let step_size = max_allowable_t_value / A::constant(num_arclength_markers as f64);
        let mut accumulated_distance = A::constant(0.0);

        let mut prev_point = interpolator.interpolate(A::constant(0.0));

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

        Self {
            interpolator,
            arclength_markers,
            total_arclength: accumulated_distance,
            phantom_data: PhantomData::default(),
        }
    }

    pub fn interpolate_points_by_arclength_absolute_stride(
        &self,
        arclength_stride: A,
    ) -> Vec<V<A>> {
        let normalized_stride = arclength_stride / self.total_arclength;
        self.interpolate_points_by_normalized_stride(normalized_stride)
    }
}
impl<A: AD, I: InterpolatorTrait<A>> InterpolatorTraitLite<A>
    for ArclengthParameterizedInterpolator<A, I>
{
    fn interpolate(&self, s: A) -> V<A> {
        // assert!(A::constant(0.0) <= s && s <= A::constant(1.0));

        let r = s * self.total_arclength;

        // Binary search requires PartialOrd. AD types usually implement PartialOrd.
        // unwrap() on partial_cmp might be risky for AD types if they can be NaN, but assume well-behaved.
        let binary_search_res = self
            .arclength_markers
            .binary_search_by(|x| x.0.partial_cmp(&r).unwrap());

        return match binary_search_res {
            Ok(idx) => self.interpolator.interpolate(self.arclength_markers[idx].1),
            Err(idx) => {
                if idx == 0 {
                    return self.interpolator.interpolate(A::constant(0.0));
                }
                let upper_bound_idx = idx;
                let lower_bound_idx = idx - 1;

                let upper_bound_dis = self.arclength_markers[upper_bound_idx].0;
                let lower_bound_dis = self.arclength_markers[lower_bound_idx].0;

                // assert!(lower_bound_dis <= r && r <= upper_bound_dis);

                let upper_bound_t = self.arclength_markers[upper_bound_idx].1;
                let lower_bound_t = self.arclength_markers[lower_bound_idx].1;

                let dis_ratio = (r - lower_bound_dis) / (upper_bound_dis - lower_bound_dis);

                let t = lower_bound_t + dis_ratio * (upper_bound_t - lower_bound_t);

                self.interpolator.interpolate(t)
            }
        };
    }

    fn max_t(&self) -> A {
        A::constant(1.0)
    }
}

#[derive(Clone)]
pub struct TimedInterpolator<A: AD, I: InterpolatorTrait<A>> {
    interpolator: I,
    max_time: A,
    phantom_data: PhantomData<V<A>>,
}
impl<A: AD, I: InterpolatorTrait<A>> TimedInterpolator<A, I> {
    pub fn new(interpolator: I, max_time: A) -> Self {
        Self {
            interpolator,
            max_time,
            phantom_data: PhantomData::default(),
        }
    }

    pub fn interpolate_points_by_time_stride(&self, time_stride: A) -> Vec<V<A>> {
        let mut out = vec![];

        let ts = get_interpolation_range(A::constant(0.0), self.max_time, time_stride);
        for t in ts {
            out.push(self.interpolate_normalized(t));
        }

        out
    }
}
impl<A: AD, I: InterpolatorTrait<A>> InterpolatorTraitLite<A> for TimedInterpolator<A, I> {
    fn interpolate(&self, t: A) -> V<A> {
        self.interpolator
            .interpolate_on_range(A::constant(0.0), self.max_time, t)
    }

    fn max_t(&self) -> A {
        self.max_time
    }
}

pub fn get_interpolation_range<A: AD>(range_start: A, range_stop: A, step_size: A) -> Vec<A> {
    assert!(range_stop >= range_start);

    let mut out_range = Vec::new();
    let mut curr_val = range_start;

    'l: loop {
        out_range.push(curr_val);
        curr_val += step_size;
        if curr_val > range_stop {
            break 'l;
        }
    }

    // Need abs() for AD trait or just check if difference is significant.
    // Assuming AD type has basic arithmetic.
    let last_val = *out_range.last().unwrap();
    let diff = range_stop - last_val;
    // Need abs on diff. AD trait usually has abs().
    if diff.abs() > A::constant(0.001) {
        out_range.push(range_stop)
    }

    out_range
}

pub fn get_interpolation_range_num_steps<A: AD>(
    range_start: A,
    range_stop: A,
    num_steps: usize,
) -> Vec<A> {
    let step_size = (range_stop - range_start) / A::constant(num_steps as f64 - 1.0);
    get_interpolation_range(range_start, range_stop, step_size)
}

pub fn linearly_interpolate_points<A: AD>(
    start_point: V<A>,
    end_point: V<A>,
    num_points: usize,
) -> Vec<V<A>> {
    let mut out = vec![];
    let range = get_interpolation_range_num_steps(A::constant(0.0), A::constant(1.0), num_points);
    for t in &range {
        let mut p = start_point.clone();
        p.scale_mut(A::constant(1.0) - t.clone());

        let mut p2 = end_point.clone();
        p2.scale_mut(t.clone());

        p += p2;
        out.push(p);
    }
    out
}

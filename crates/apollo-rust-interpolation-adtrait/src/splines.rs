use crate::InterpolatorTraitLite;
use ad_trait::AD;
use apollo_rust_linalg_adtrait::V;
use nalgebra::DVector;
use std::collections::VecDeque;

#[derive(Clone, Debug)]
pub struct InterpolatingSpline<A: AD> {
    control_points: Vec<V<A>>,
    spline_segment_a_coefficients: Vec<Vec<V<A>>>,
    spline_type: InterpolatingSplineType,
    num_spline_segments: usize,
}
impl<A: AD> InterpolatingSpline<A> {
    pub fn new(control_points: Vec<V<A>>, spline_type: InterpolatingSplineType) -> Self {
        let num_points = control_points.len();
        assert!(num_points > 0);

        assert_eq!(
            (num_points - spline_type.num_overlap_between_segments())
                % (spline_type.num_control_points_per_segment()
                    - spline_type.num_overlap_between_segments()),
            0
        );

        let num_spline_segments = (num_points - spline_type.num_overlap_between_segments())
            / (spline_type.num_control_points_per_segment()
                - spline_type.num_overlap_between_segments());
        let control_point_dim = control_points[0].len();

        let zeros = <V<A>>::from_element(control_point_dim, A::constant(0.0));

        let spline_segment_a_coefficients =
            vec![vec![zeros; spline_type.num_control_points_per_segment()]; num_spline_segments];

        let mut out_self = Self {
            control_points,
            spline_segment_a_coefficients,
            spline_type,
            num_spline_segments,
        };

        for i in 0..num_spline_segments {
            out_self.calculate_a_vector_coefficients(i);
        }

        out_self
    }

    #[inline]
    pub fn update_control_point(&mut self, idx: usize, control_point: V<A>) {
        self.control_points[idx] = control_point;
        let spline_segment_idxs = self.map_control_point_idx_to_spline_segment_idxs(idx);
        for s in spline_segment_idxs {
            self.calculate_a_vector_coefficients(s);
        }
    }

    #[inline]
    pub fn spline_interpolate(&self, t: A) -> V<A> {
        let max_t = self.max_allowable_t_value();
        if t == max_t {
            return self.spline_interpolate(t - A::constant(0.00000001));
        }

        // assert!(t >= A::constant(0.0));
        // t.floor() in AD? usually yes.
        let rt = t.fract();
        // Index needs to be usize, so we need to extract value from AD type if it's constant structure,
        // but wait, if 't' determines the segment index, then the gradient flows through the CHOICE of segment?
        // No, usually index is discrete. We can't differentiate through distinct integer steps easily without smooth switching.
        // Standard splines are C2 continuous (usually), so gradients should be fine at boundaries.
        // However, we need to convert t to integer index. AD types don't always implement `to_f64()` easily inside the graph.
        // But for typical AD (Forward/Reverse), we use the primal value to decide control flow.
        // If A is f64-like wrapper, we might have standard methods.
        // Let's assume A behaves like a float.

        // We need to get the "value" of A to determine index.
        // This effectively treats the index choice as constant with respect to derivatives at the point of evaluation,
        // which matches the mathematical definition (piecewise polynomial).
        // Since it's continuous, the derivative matches at boundaries.
        // But in Rust, `t.floor()` returns `A`. We need `usize`.
        // We might need a trait method on A or convert via cast if possible.
        // Usually AD libraries provide `to_f64()` or similar.
        // Let's check ad_trait. It has `to_constant_ad()` but getting the underlying `f64` might require `to_constant()` if available.
        // Actually, `t.floor()` gives `A`. If `A` is `ForwardAD<f64>`, it's still a struct.
        // We need to cast.
        // Since we are porting `apollo-rust-interpolation`, let's see how we can handle this.
        // One way is `t.floor().to_f64()`?
        // ad_trait `AD` requirement usually implies `RealField` or similar.

        // For now, let's try to convert via string parse or specialized trait if available? No that's slow.
        // If A: RealField, we have `to_f64(&self)`.

        // Let's try `t.floor()` returning A, then how to get usize?
        // Maybe A has `to_constant()` to return primitive?
        // Inspecting ad_trait usage elsewhere might help.
        // But `nalgebra::RealField` has `to_subset`.

        // HACK: For now assume we can't extract f64 easily without `RealField`.
        // But wait, `ad_trait` likely implements `RealField`.
        // nalgebra's `RealField` implies `Copy`, `PartialOrd`, etc.
        // Let's assume we can cast.

        // Actually, let's look at `splines.rs` logic: `let spline_segment_idx = t.floor() as usize;`
        // `t.floor()` is `A`. `as usize` won't work on a struct.
        // We need the integer value.
        // Assuming we are in a context where we can get the value.
        // If A is `f64`, it works. If A is `ForwardAD`, it wraps `f64`.
        // We probably need `nalgebra::convert(t.floor())`.

        // Let's try `let spline_segment_idx_a = t.floor();`.
        // We need to extract the value.
        // Let's assume we can use `approx` or safe cast via `to_subset` or similar.
        // Or assume A implements `to_f64()` if defined in AD trait?
        // Checking `ad_trait` definition would be ideal but I don't have it open.
        // I will trust that for now and if it fails compilation I'll fix.
        // One safe bet: `t` might be castable if it implements `buffer` access?

        // Wait, typical pattern in these crates:
        // `let idx = t.floor().to_subset().unwrap() as usize;` if `A` impls `SubsetOf<f64>`.
        // Or `let val: f64 = nalgebra::convert(t.floor());`

        // I'll try `nalgebra::convert(t.floor())`.

        // Re-reading usage.

        // Loop to find index since we can't easily cast AD to usize
        let mut spline_segment_idx = self.num_spline_segments - 1;
        for i in 0..self.num_spline_segments {
            if t < A::constant((i + 1) as f64) {
                spline_segment_idx = i;
                break;
            }
        }

        assert!(spline_segment_idx < self.num_spline_segments);

        let a_vecs = &self.spline_segment_a_coefficients[spline_segment_idx];

        // Horner's method
        let mut out = a_vecs
            .last()
            .expect("spline segments must have coefficients")
            .clone();
        for a in a_vecs.iter().rev().skip(1) {
            out.scale_mut(rt);
            out += a;
        }

        return out;
    }

    #[inline]
    pub fn map_control_point_idx_to_spline_segment_idxs(
        &self,
        control_point_idx: usize,
    ) -> Vec<usize> {
        assert!(control_point_idx < self.control_points.len());

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let a = control_point_idx % (num_control_points_per_segment - num_overlap_between_segments);
        let b = control_point_idx / (num_control_points_per_segment - num_overlap_between_segments);

        let dis_from_segment_edge_option1 = a;
        let dis_from_segment_edge_option2 = num_control_points_per_segment - a;
        let dis_from_either_edge_of_segment =
            usize::min(dis_from_segment_edge_option1, dis_from_segment_edge_option2);

        if dis_from_either_edge_of_segment >= num_overlap_between_segments {
            return vec![b];
        }

        assert_ne!(dis_from_segment_edge_option1, dis_from_segment_edge_option2);
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2 && b == 0 {
            return vec![b];
        }
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2
            && b == self.num_spline_segments
        {
            return vec![b - 1];
        }
        if dis_from_segment_edge_option2 < dis_from_segment_edge_option1
            && b == self.num_spline_segments
        {
            return vec![b - 1];
        }

        return vec![b - 1, b];
    }

    #[inline]
    pub fn calculate_a_vector_coefficients(&mut self, spline_segment_idx: usize) {
        let control_point_idxs =
            self.map_spline_segment_idx_to_control_point_idxs(spline_segment_idx);
        let control_point_refs: Vec<&V<A>> = control_point_idxs
            .iter()
            .map(|x| &self.control_points[*x])
            .collect();
        let basis_matrix = self.spline_type.basis_matrix();

        // Basis matrix is f64, but we need A?
        // We can do `A::constant(val)` for each element.

        let basis_matrix_ad: Vec<Vec<A>> = basis_matrix
            .iter()
            .map(|row| row.iter().map(|x| A::constant(*x)).collect())
            .collect();

        // Need to change signature of generic helper or adapt it.
        // It's local fn, let's just implement logic here or adapt.

        let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix_ad);
        self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
    }

    #[inline]
    pub fn map_spline_segment_idx_to_control_point_idxs(
        &self,
        spline_segment_idx: usize,
    ) -> Vec<usize> {
        assert!(
            spline_segment_idx < self.num_spline_segments,
            "idx {}, num_spline_segments {}",
            spline_segment_idx,
            self.num_spline_segments
        );

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let start =
            (num_control_points_per_segment - num_overlap_between_segments) * spline_segment_idx;

        let mut out_vec = vec![];
        for i in 0..num_control_points_per_segment {
            out_vec.push(start + i);
        }

        out_vec
    }

    #[inline]
    pub fn max_allowable_t_value(&self) -> A {
        return A::constant(self.num_spline_segments as f64);
    }
}
impl<A: AD> InterpolatorTraitLite<A> for InterpolatingSpline<A> {
    fn interpolate(&self, t: A) -> V<A> {
        self.spline_interpolate(t)
    }

    fn max_t(&self) -> A {
        self.max_allowable_t_value()
    }
}

fn calculate_a_vector_coefficients_generic<A: AD>(
    p: &Vec<&V<A>>,
    basis_matrix: &Vec<Vec<A>>,
) -> Vec<V<A>> {
    let mut out_vec = vec![];
    for row in basis_matrix {
        let mut a = <V<A>>::from_element(p[0].len(), A::constant(0.0));
        for (i, value) in row.iter().enumerate() {
            // a += p[i] * *value;
            // Vector * Scalar
            a += p[i].clone() * value.clone();
        }
        out_vec.push(a);
    }
    out_vec
}

#[derive(Clone, Debug, Copy)]
pub enum InterpolatingSplineType {
    Linear,
    Quadratic,
    HermiteCubic,
    NaturalCubic,
    CardinalCubic { w: f64 },
    BezierCubic,
}
impl InterpolatingSplineType {
    #[inline(always)]
    pub fn num_control_points_per_segment(&self) -> usize {
        match self {
            InterpolatingSplineType::Linear => 2,
            InterpolatingSplineType::Quadratic => 3,
            InterpolatingSplineType::HermiteCubic => 4,
            InterpolatingSplineType::NaturalCubic => 4,
            InterpolatingSplineType::CardinalCubic { .. } => 4,
            InterpolatingSplineType::BezierCubic => 4,
        }
    }

    #[inline(always)]
    pub fn num_overlap_between_segments(&self) -> usize {
        match self {
            InterpolatingSplineType::Linear => 1,
            InterpolatingSplineType::Quadratic => 1,
            InterpolatingSplineType::HermiteCubic => 2,
            InterpolatingSplineType::NaturalCubic => 1,
            InterpolatingSplineType::CardinalCubic { .. } => 2,
            InterpolatingSplineType::BezierCubic => 2,
        }
    }

    pub fn basis_matrix(&self) -> Vec<Vec<f64>> {
        match self {
            InterpolatingSplineType::Linear => vec![vec![1.0, 0.0], vec![-1.0, 1.0]],
            InterpolatingSplineType::Quadratic => vec![
                vec![1.0, 0.0, 0.0],
                vec![-3.0, 4.0, -1.0],
                vec![2.0, -4.0, 2.0],
            ],
            InterpolatingSplineType::HermiteCubic => vec![
                vec![1.0, 0.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0, 0.0],
                vec![-3.0, -2.0, 3.0, -1.0],
                vec![2.0, 1.0, -2.0, 1.0],
            ],
            InterpolatingSplineType::NaturalCubic => vec![
                vec![1.0, 0.0, 0.0, 0.0],
                vec![0.0, 1.0, 0.0, 0.0],
                vec![0.0, 0.0, 0.5, 0.0],
                vec![-1.0, -1.0, -0.5, 1.0],
            ],
            InterpolatingSplineType::CardinalCubic { w } => {
                let w = *w;
                vec![
                    vec![0.0, 1.0, 0.0, 0.0],
                    vec![(w - 1.0) / 2.0, 0.0, (1.0 - w) / 2.0, 0.0],
                    vec![1.0 - w, -0.5 * (w + 5.0), w + 2.0, (w - 1.0) / 2.0],
                    vec![
                        (w - 1.0) / 2.0,
                        (w + 3.0) / 2.0,
                        -0.5 * (w + 3.0),
                        (1.0 - w) / 2.0,
                    ],
                ]
            }
            InterpolatingSplineType::BezierCubic => vec![
                vec![1.0, 0.0, 0.0, 0.0],
                vec![-3.0, 3.0, 0.0, 0.0],
                vec![3.0, -6.0, 3.0, 0.0],
                vec![-1.0, 3.0, -3.0, 1.0],
            ],
        }
    }
}

pub fn get_interpolation_range<A: AD>(range_start: A, range_stop: A, step_size: A) -> Vec<A> {
    assert!(range_stop >= range_start);
    let mut out_range = vec![];
    let mut curr_val = range_start;

    while curr_val <= range_stop {
        out_range.push(curr_val);
        curr_val += step_size;
    }

    if (range_stop - *out_range.last().unwrap()).abs() > A::constant(0.001) {
        out_range.push(range_stop);
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

#[derive(Clone, Debug)]
pub struct BSpline<A: AD> {
    control_points: Vec<V<A>>,
    knot_vector: Vec<f64>, // Knot vector remains f64 usually
    k: usize,
}

impl<A: AD> BSpline<A> {
    pub fn new(
        control_points: Vec<V<A>>,
        k: usize,
        interpolate_first_control_point: bool,
        interpolate_last_control_point: bool,
    ) -> Self {
        assert!(k > 1);
        let n = control_points.len() - 1;
        let p = k - 1;
        let m = n + k + 1;
        let mut knot_vector = vec![0.0; m];

        if interpolate_first_control_point && interpolate_last_control_point {
            for i in 0..m {
                if i < k {
                    knot_vector[i] = 0.0;
                } else if i >= m - k {
                    knot_vector[i] = (n - p + 1) as f64;
                } else {
                    knot_vector[i] = (i - k + 1) as f64;
                }
            }
        } else if interpolate_first_control_point {
            for i in 0..m {
                if i < k {
                    knot_vector[i] = 0.0;
                } else {
                    knot_vector[i] = (i - k + 1) as f64;
                }
            }
        } else if interpolate_last_control_point {
            for i in 0..m {
                if i < m - k {
                    knot_vector[i] = i as f64;
                } else {
                    knot_vector[i] = (m - k) as f64;
                }
            }
        } else {
            for i in 0..m {
                knot_vector[i] = i as f64;
            }
        }

        Self {
            control_points,
            knot_vector,
            k,
        }
    }

    #[inline]
    fn cox_de_boor_recurrence(&self, i: usize, k: usize, t: A) -> A {
        // This recursive version is slow, we should use the fast one,
        // but keep it as naive fallback or implementation reference.
        // However, to keep it simple and performant given the previous task,
        // I will port the naive one too as fallback.

        // Converting A to f64 for comparison with knots might be needed if knots are f64.
        // Or promote knots to A?
        // Let's promote knots to A for logic.

        let knot_i = A::constant(self.knot_vector[i]);
        let knot_i_plus_1 = A::constant(self.knot_vector[i + 1]);
        let knot_len_minus_1 = A::constant(self.knot_vector[self.knot_vector.len() - 1]);
        let knot_i_plus_k_minus_1 = A::constant(self.knot_vector[i + k - 1]);
        let knot_i_plus_k = A::constant(self.knot_vector[i + k]);

        if k == 1 {
            // Need boolean logic in AD?
            // If A wraps a value, we can use standard comparison if it impls PartialOrd.
            // But branching on A in a differentiable graph requires care (switching).
            // For simple AD like Forward/Reverse, Primal value decides branch.

            if (knot_i <= t && t < knot_i_plus_1)
                || (t == knot_len_minus_1 && i == self.control_points.len() - 1)
            {
                A::constant(1.0)
            } else {
                A::constant(0.0)
            }
        } else {
            let denom1 = knot_i_plus_k_minus_1 - knot_i;
            let denom2 = knot_i_plus_k - knot_i_plus_1;

            let c0 = if denom1 != A::constant(0.0) {
                (t - knot_i) / denom1 * self.cox_de_boor_recurrence(i, k - 1, t)
            } else {
                A::constant(0.0)
            };

            let c1 = if denom2 != A::constant(0.0) {
                (knot_i_plus_k - t) / denom2 * self.cox_de_boor_recurrence(i + 1, k - 1, t)
            } else {
                A::constant(0.0)
            };

            c0 + c1
        }
    }

    #[inline]
    pub fn bspline_interpolate_naive(&self, t: A) -> V<A> {
        let n = self.control_points.len() - 1;
        let _p = self.k - 1;
        // Optimization: only sum over relevant basis functions?
        // Naive sums all.

        let zeros = <V<A>>::from_element(self.control_points[0].len(), A::constant(0.0));
        let mut out_sum = zeros;

        let knot_last = A::constant(self.knot_vector[self.knot_vector.len() - 1]);

        let _t = if t == knot_last {
            t - A::constant(1e-12)
        } else {
            t
        };
        for i in 0..=n {
            let basis = self.cox_de_boor_recurrence(i, self.k, _t);
            // control_points[i] * basis
            // DVector * Scalar
            out_sum += self.control_points[i].clone() * basis.clone();
        }
        out_sum
    }

    #[inline]
    pub fn bspline_interpolate_fast(&self, t: A) -> V<A> {
        let p = self.k - 1;

        let t_max_val = self.knot_vector[self.knot_vector.len() - 1];
        let t_max = A::constant(t_max_val);

        // Handle t exactly at end
        let diff = (t - t_max).abs();
        let _t = if diff < A::constant(1e-12) {
            t_max - A::constant(1e-12)
        } else {
            t
        };

        // Linear scan for knot span.
        // Needs t's value.
        // Assuming we can extract f64 from A for index logic, or use comparison.
        // But loop variable `i` is usize.

        let mut knot_span_idx = self.knot_vector.len() - 1;
        for i in 0..self.knot_vector.len() - 1 {
            let k_i = A::constant(self.knot_vector[i]);
            let k_i1 = A::constant(self.knot_vector[i + 1]);

            if k_i <= _t && _t < k_i1 {
                knot_span_idx = i;
                break;
            }
        }

        if knot_span_idx < p || knot_span_idx + p >= self.knot_vector.len() {
            return self.bspline_interpolate_naive(_t);
        }

        // Algorithm A2.2
        let mut left = vec![A::constant(0.0); p + 1];
        let mut right = vec![A::constant(0.0); p + 1];
        let mut n_basis = vec![A::constant(0.0); p + 1];
        n_basis[0] = A::constant(1.0);

        for j in 1..=p {
            left[j] = _t - A::constant(self.knot_vector[knot_span_idx + 1 - j]);
            right[j] = A::constant(self.knot_vector[knot_span_idx + j]) - _t;
            let mut saved = A::constant(0.0);
            for r in 0..j {
                let temp = n_basis[r] / (right[r + 1] + left[j - r]);
                n_basis[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n_basis[j] = saved;
        }

        let start_cp_idx = knot_span_idx.saturating_sub(p);

        let mut out = self.control_points[start_cp_idx].clone();
        out *= n_basis[0].clone();

        for r in 1..=p {
            let cp_idx = start_cp_idx + r;
            if cp_idx < self.control_points.len() {
                // out += self.control_points[cp_idx] * n_basis[r]
                // out.axpy(n_basis[r], &self.control_points[cp_idx], 1.0);
                // DVector generic might not support axpy with AD types directly if traits differ.
                // Naive: out = out + cp * basis
                out += self.control_points[cp_idx].clone() * n_basis[r].clone();
            }
        }

        out
    }

    #[inline(always)]
    pub fn update_control_point(&mut self, idx: usize, control_point: V<A>) {
        self.control_points[idx] = control_point;
    }

    #[inline(always)]
    pub fn control_points(&self) -> &Vec<V<A>> {
        &self.control_points
    }

    #[inline(always)]
    fn max_allowable_t_value(&self) -> A {
        A::constant(self.knot_vector[self.knot_vector.len() - self.k])
    }

    #[inline(always)]
    fn min_allowable_t_value(&self) -> A {
        A::constant(self.knot_vector[self.k - 1])
    }
}

impl<A: AD> InterpolatorTraitLite<A> for BSpline<A> {
    #[inline]
    fn interpolate(&self, t: A) -> V<A> {
        self.bspline_interpolate_fast(t)
    }

    #[inline(always)]
    fn max_t(&self) -> A {
        self.max_allowable_t_value()
    }

    fn interpolate_normalized(&self, u: A) -> V<A> {
        let t_start = self.min_allowable_t_value();
        let t_end = self.max_t();
        let t = t_start + (t_end - t_start) * u;
        // Handle floating-point precision
        let t = if t > t_end { t_end } else { t };
        self.interpolate(t)
    }
}

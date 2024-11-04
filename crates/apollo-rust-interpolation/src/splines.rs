use nalgebra::DVector;
use crate::InterpolatorTraitLite;

#[derive(Clone, Debug)]
pub struct InterpolatingSpline {
    control_points: Vec<DVector<f64>>,
    spline_segment_a_coefficients: Vec<Vec<DVector<f64>>>,
    spline_type: InterpolatingSplineType,
    num_spline_segments: usize
}
impl InterpolatingSpline {
    pub fn new(control_points: Vec<DVector<f64>>, spline_type: InterpolatingSplineType) -> Self {
        let num_points = control_points.len();
        assert!(num_points > 0);

        assert_eq!(
            (num_points - spline_type.num_overlap_between_segments())
                % (spline_type.num_control_points_per_segment() - spline_type.num_overlap_between_segments()),
            0
        );

        let num_spline_segments = (num_points - spline_type.num_overlap_between_segments())
            / (spline_type.num_control_points_per_segment() - spline_type.num_overlap_between_segments());
        let control_point_dim = control_points[0].len();

        let spline_segment_a_coefficients = vec![
            vec![DVector::zeros(control_point_dim); spline_type.num_control_points_per_segment()];
            num_spline_segments
        ];

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
    pub fn update_control_point(&mut self, idx: usize, control_point: DVector<f64>) {
        self.control_points[idx] = control_point;
        let spline_segment_idxs = self.map_control_point_idx_to_spline_segment_idxs(idx);
        for s in spline_segment_idxs {
            self.calculate_a_vector_coefficients(s);
        }
    }

    #[inline]
    pub fn spline_interpolate(&self, t: f64) -> DVector<f64> {
        if t == self.max_allowable_t_value() {
            return self.spline_interpolate(t - 0.00000001);
        }

        assert!(t >= 0.0);
        let rt = t.fract();
        let spline_segment_idx = t.floor() as usize;
        assert!(spline_segment_idx < self.num_spline_segments);

        let a_vecs = &self.spline_segment_a_coefficients[spline_segment_idx];

        let mut out = DVector::zeros(a_vecs[0].len());
        for (i, a) in a_vecs.iter().enumerate() {
            out += a * rt.powi(i as i32);
        }

        return out;
    }

    #[inline]
    pub fn map_control_point_idx_to_spline_segment_idxs(&self, control_point_idx: usize) -> Vec<usize> {
        assert!(control_point_idx < self.control_points.len());

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let a = control_point_idx % (num_control_points_per_segment - num_overlap_between_segments);
        let b = control_point_idx / (num_control_points_per_segment - num_overlap_between_segments);

        let dis_from_segment_edge_option1 = a;
        let dis_from_segment_edge_option2 = num_control_points_per_segment - a;
        let dis_from_either_edge_of_segment = usize::min(dis_from_segment_edge_option1, dis_from_segment_edge_option2);

        if dis_from_either_edge_of_segment >= num_overlap_between_segments {
            return vec![b];
        }

        assert_ne!(dis_from_segment_edge_option1, dis_from_segment_edge_option2);
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2 && b == 0 {
            return vec![b];
        }
        if dis_from_segment_edge_option1 < dis_from_segment_edge_option2 && b == self.num_spline_segments {
            return vec![b - 1];
        }
        if dis_from_segment_edge_option2 < dis_from_segment_edge_option1 && b == self.num_spline_segments {
            return vec![b - 1];
        }

        return vec![b - 1, b];
    }

    #[inline]
    pub fn calculate_a_vector_coefficients(&mut self, spline_segment_idx: usize) {
        let control_point_idxs = self.map_spline_segment_idx_to_control_point_idxs(spline_segment_idx);
        let control_point_refs: Vec<&DVector<f64>> = control_point_idxs.iter().map(|x| &self.control_points[*x]).collect();
        let basis_matrix = self.spline_type.basis_matrix();

        let a_vecs = calculate_a_vector_coefficients_generic(&control_point_refs, &basis_matrix);
        self.spline_segment_a_coefficients[spline_segment_idx] = a_vecs;
    }

    #[inline]
    pub fn map_spline_segment_idx_to_control_point_idxs(&self, spline_segment_idx: usize) -> Vec<usize> {
        assert!(spline_segment_idx < self.num_spline_segments, "idx {}, num_spline_segments {}", spline_segment_idx, self.num_spline_segments);

        let num_control_points_per_segment = self.spline_type.num_control_points_per_segment();
        let num_overlap_between_segments = self.spline_type.num_overlap_between_segments();

        let start = (num_control_points_per_segment - num_overlap_between_segments) * spline_segment_idx;

        let mut out_vec = vec![];
        for i in 0..num_control_points_per_segment {
            out_vec.push(start + i);
        }

        out_vec
    }

    #[inline]
    pub fn max_allowable_t_value(&self) -> f64 {
        return self.num_spline_segments as f64;
    }
}
impl InterpolatorTraitLite for InterpolatingSpline {
    fn interpolate(&self, t: f64) -> DVector<f64> {
        self.spline_interpolate(t)
    }

    fn max_t(&self) -> f64 {
        self.max_allowable_t_value()
    }

    fn interpolate_on_range(&self, range_start: f64, range_stop: f64, t: f64) -> DVector<f64> {
        assert!(range_start <= t && t <= range_stop);
        let ratio = (t - range_start) / (range_stop - range_start);
        self.interpolate_normalized(ratio)
    }

    fn interpolate_normalized(&self, u: f64) -> DVector<f64> {
        assert!(0.0 <= u && u <= 1.0);
        let ratio = self.max_t() * u;
        self.interpolate(ratio)
    }

    fn interpolate_points_by_num_points(&self, num_points: usize) -> Vec<DVector<f64>> {
        let mut out = vec![];
        let ts = get_interpolation_range_num_steps(0.0, 1.0, num_points);
        for t in ts {
            out.push(self.interpolate_normalized(t));
        }
        out
    }

    fn interpolate_points_by_normalized_stride(&self, stride_length: f64) -> Vec<DVector<f64>> {
        let mut out = vec![];
        let ts = get_interpolation_range(0.0, 1.0, stride_length);
        for t in ts {
            out.push(self.interpolate_normalized(t));
        }
        out
    }
}

fn calculate_a_vector_coefficients_generic(
    p: &Vec<&DVector<f64>>,
    basis_matrix: &Vec<Vec<f64>>
) -> Vec<DVector<f64>> {
    let mut out_vec = vec![];
    for row in basis_matrix {
        let mut a = DVector::zeros(p[0].len());
        for (i, value) in row.iter().enumerate() {
            a += p[i] * *value;
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
            InterpolatingSplineType::Quadratic => vec![vec![1.0, 0.0, 0.0], vec![-3.0, 4.0, -1.0], vec![2.0, -4.0, 2.0]],
            InterpolatingSplineType::HermiteCubic => vec![vec![1.0, 0.0, 0.0, 0.0], vec![0.0, 1.0, 0.0, 0.0], vec![-3.0, -2.0, 3.0, -1.0], vec![2.0, 1.0, -2.0, 1.0]],
            InterpolatingSplineType::NaturalCubic => vec![vec![1.0, 0.0, 0.0, 0.0], vec![0.0, 1.0, 0.0, 0.0], vec![0.0, 0.0, 0.5, 0.0], vec![-1.0, -1.0, -0.5, 1.0]],
            InterpolatingSplineType::CardinalCubic { w } => {
                let w = *w;
                vec![
                    vec![0.0, 1.0, 0.0, 0.0],
                    vec![(w - 1.0) / 2.0, 0.0, (1.0 - w) / 2.0, 0.0],
                    vec![1.0 - w, -0.5 * (w + 5.0), w + 2.0, (w - 1.0) / 2.0],
                    vec![(w - 1.0) / 2.0, (w + 3.0) / 2.0, -0.5 * (w + 3.0), (1.0 - w) / 2.0],
                ]
            }
            InterpolatingSplineType::BezierCubic => vec![vec![1.0, 0.0, 0.0, 0.0], vec![-3.0, 3.0, 0.0, 0.0], vec![3.0, -6.0, 3.0, 0.0], vec![-1.0, 3.0, -3.0, 1.0]],
        }
    }
}

pub fn get_interpolation_range(range_start: f64, range_stop: f64, step_size: f64) -> Vec<f64> {
    assert!(range_stop >= range_start);
    let mut out_range = vec![];
    let mut curr_val = range_start;

    while curr_val <= range_stop {
        out_range.push(curr_val);
        curr_val += step_size;
    }

    if (range_stop - out_range.last().unwrap()).abs() > 0.001 {
        out_range.push(range_stop);
    }

    out_range
}

pub fn get_interpolation_range_num_steps(range_start: f64, range_stop: f64, num_steps: usize) -> Vec<f64> {
    let step_size = (range_stop - range_start) / (num_steps as f64 - 1.0);
    get_interpolation_range(range_start, range_stop, step_size)
}

#[derive(Clone, Debug)]
pub struct BSpline {
    control_points: Vec<DVector<f64>>,
    knot_vector: Vec<f64>,
    k: usize
}

impl BSpline {
    pub fn new(
        control_points: Vec<DVector<f64>>,
        k: usize,
        interpolate_first_control_point: bool,
        interpolate_last_control_point: bool,
    ) -> Self {
        assert!(k > 1);
        let n = control_points.len() - 1; // Number of control points minus one
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
            k
        }
    }

    #[inline]
    pub fn cox_de_boor_recurrence(&self, i: usize, k: usize, t: f64) -> f64 {
        if k == 1 {
            if (self.knot_vector[i] <= t && t < self.knot_vector[i + 1])
                || (t == self.knot_vector[self.knot_vector.len() - 1] && i == self.control_points.len() - 1)
            {
                1.0
            } else {
                0.0
            }
        } else {
            let denom1 = self.knot_vector[i + k - 1] - self.knot_vector[i];
            let denom2 = self.knot_vector[i + k] - self.knot_vector[i + 1];

            let c0 = if denom1 != 0.0 {
                (t - self.knot_vector[i]) / denom1 * self.cox_de_boor_recurrence(i, k - 1, t)
            } else {
                0.0
            };

            let c1 = if denom2 != 0.0 {
                (self.knot_vector[i + k] - t) / denom2 * self.cox_de_boor_recurrence(i + 1, k - 1, t)
            } else {
                0.0
            };

            c0 + c1
        }
    }

    #[inline]
    fn bspline_interpolate(&self, t: f64) -> DVector<f64> {
        let n = self.control_points.len() - 1;
        let mut out_sum = DVector::zeros(self.control_points[0].len());

        for i in 0..=n {
            let basis = self.cox_de_boor_recurrence(i, self.k, t);
            out_sum += self.control_points[i].clone() * basis;
        }

        out_sum
    }

    #[inline(always)]
    pub fn update_control_point(&mut self, idx: usize, control_point: DVector<f64>) {
        self.control_points[idx] = control_point;
    }

    #[inline(always)]
    pub fn control_points(&self) -> &Vec<DVector<f64>> {
        &self.control_points
    }

    #[inline(always)]
    fn max_allowable_t_value(&self) -> f64 {
        self.knot_vector[self.knot_vector.len() - self.k]
    }

    #[inline(always)]
    fn min_allowable_t_value(&self) -> f64 {
        self.knot_vector[self.k - 1]
    }
}

impl InterpolatorTraitLite for BSpline {
    #[inline]
    fn interpolate(&self, t: f64) -> DVector<f64> {
        self.bspline_interpolate(t)
    }

    #[inline(always)]
    fn max_t(&self) -> f64 {
        self.max_allowable_t_value()
    }

    fn interpolate_normalized(&self, u: f64) -> DVector<f64> {
        assert!(0.0 <= u && u <= 1.0);
        let t_start = self.min_allowable_t_value();
        let t_end = self.max_t();
        let t = t_start + (t_end - t_start) * u;
        // Handle floating-point precision
        let t = if t > t_end { t_end } else { t };
        self.interpolate(t)
    }
}

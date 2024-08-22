use std::cmp::Ordering;
use std::fmt::Debug;
use nalgebra::DMatrix;
use parry3d_f64::query::{Contact, contact, distance, intersection_test};
use parry3d_f64::shape::Shape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::offset_shape::OffsetShape;
use crate::{ProximityLossFunction, ToIntersectionResult, ToProximityValue};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DoubleGroupProximityQueryMode {
    AllPossiblePairs,
    SkipSymmetricalPairs,
    SubsetOfPairs(Vec<(usize, usize)>)
}

#[derive(Clone, Debug)]
pub struct DoubleGroupProximityQueryOutput<T: Clone + Debug> {
    pub outputs: Vec<T>,
    pub shape_idxs: Vec<(usize, usize)>,
    pub num_ground_truth_checks: usize
}
impl<T: Clone + Debug> DoubleGroupProximityQueryOutput<T> {
    pub fn new(outputs: Vec<T>, shape_idxs: Vec<(usize, usize)>, num_ground_truth_checks: usize) -> Self {
        Self { outputs, shape_idxs, num_ground_truth_checks }
    }
}

impl ToIntersectionResult for DoubleGroupProximityQueryOutput<bool> {
    fn to_intersection_result(&self) -> bool {
        for res in &self.outputs { if *res { return true; } }
        false
    }
}
impl ToIntersectionResult for DoubleGroupProximityQueryOutput<f64> {
    fn to_intersection_result(&self) -> bool {
        for res in &self.outputs { if *res <= 0.0 { return true; } }
        false
    }
}
impl ToIntersectionResult for DoubleGroupProximityQueryOutput<Option<Contact>> {
    fn to_intersection_result(&self) -> bool {
        for res in &self.outputs {
            match res {
                None => { }
                Some(res) => { if res.dist <= 0.0 { return true; } }
            }
        }
        false
    }
}

impl ToProximityValue for DoubleGroupProximityQueryOutput<f64> {
    fn to_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.outputs.iter().for_each(|x| out += loss.loss(*x).powf(p_norm));

        out.powf(1.0/p_norm)
    }
}
impl ToProximityValue for DoubleGroupProximityQueryOutput<Option<Contact>> {
    fn to_proximity_value(&self, loss: &ProximityLossFunction, p_norm: f64) -> f64 {
        let mut out = 0.0;

        self.outputs.iter().for_each(|x| {
            match x {
                None => { }
                Some(x) => {
                    out += loss.loss(x.dist).powf(p_norm)
                }
            }
        });

        out.powf(1.0/p_norm)
    }
}

pub trait ConvertToAverageDistancesTrait {
    fn convert_to_average_distances_in_place(&mut self, average_distances: &DMatrix<f64>);
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Self;
}
impl ConvertToAverageDistancesTrait for DoubleGroupProximityQueryOutput<f64> {
    fn convert_to_average_distances_in_place(&mut self, average_distances: &DMatrix<f64>) {
        self.outputs.iter_mut().zip(self.shape_idxs.iter()).for_each(|(x, (i, j))| {
            // let average = average_distances[(*i, *j)].max(0.1);
            let mut average = average_distances[(*i, *j)];
            if average < 0.0 { average = 1.0; }
            *x /= average;
        });
    }

    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Self {
        let outputs: Vec<f64> = self.outputs.iter().zip(self.shape_idxs.iter()).map(|(x, (i, j))| {
            // let average = average_distances[(*i, *j)].max(0.0001);
            let mut average = average_distances[(*i, *j)];
            if average < 0.0 { average = 1.0; }
            *x / average
        }).collect();

        Self {
            outputs,
            shape_idxs: self.shape_idxs.clone(),
            num_ground_truth_checks: self.num_ground_truth_checks,
        }
    }
}
impl ConvertToAverageDistancesTrait for DoubleGroupProximityQueryOutput<Option<Contact>> {
    fn convert_to_average_distances_in_place(&mut self, average_distances: &DMatrix<f64>) {
        self.outputs.iter_mut().zip(self.shape_idxs.iter()).for_each(|(x, (i, j))| {
            match x {
                None => { }
                Some(x) => {
                    // let average = average_distances[(*i, *j)].max(0.0001);
                    let mut average = average_distances[(*i, *j)];
                    if average < 0.0 { average = 1.0; }
                    x.dist /= average;
                }
            }
        });
    }

    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Self {
        let outputs: Vec<Option<Contact>> = self.outputs.iter().zip(self.shape_idxs.iter()).map(|(x, (i, j))| {
            match x {
                None => { None }
                Some(x) => {
                    let mut c = x.clone();
                    // let average = average_distances[(*i, *j)].max(0.0001);
                    let mut average = average_distances[(*i, *j)];
                    if average < 0.0 { average = 1.0; }
                    c.dist /= average;
                    Some(c)
                }
            }
        }).collect();

        Self {
            outputs,
            shape_idxs: self.shape_idxs.clone(),
            num_ground_truth_checks: self.num_ground_truth_checks,
        }
    }
}

pub trait SortDoubleGroupProximityQueryOutputTrait {
    fn sort(&self) -> Self;
}
impl SortDoubleGroupProximityQueryOutputTrait for DoubleGroupProximityQueryOutput<f64> {
    fn sort(&self) -> Self {
        let mut idxs: Vec<usize> = (0..self.outputs.len()).collect();

        idxs.sort_by(|x, y| self.outputs[*x].partial_cmp(&self.outputs[*y]).unwrap() );
        let shape_idxs = idxs.iter().map(|x| self.shape_idxs[*x].clone() ).collect();
        let outputs = idxs.iter().map(|x| self.outputs[*x].clone()).collect();

        DoubleGroupProximityQueryOutput {
            outputs,
            shape_idxs,
            num_ground_truth_checks: self.num_ground_truth_checks,
        }
    }
}
impl SortDoubleGroupProximityQueryOutputTrait for DoubleGroupProximityQueryOutput<Option<Contact>> {
    fn sort(&self) -> Self {
        let mut idxs: Vec<usize> = (0..self.outputs.len()).collect();

        idxs.sort_by(|x, y| {
           match &self.outputs[*x] {
               None => {
                  match &self.outputs[*y] {
                      None => { Ordering::Equal }
                      Some(_) => { Ordering::Greater }
                  }
               }
               Some(cx) => {
                   match &self.outputs[*y] {
                       None => { Ordering::Less }
                       Some(cy) => {
                           if cx.dist.is_nan() || cy.dist.is_nan() { return Ordering::Equal }
                           return cx.dist.partial_cmp(&cy.dist).expect(&format!("error {}, {}", cx.dist, cy.dist))
                       }
                   }
               }
           }
        });

        let shape_idxs = idxs.iter().map(|x| self.shape_idxs[*x].clone() ).collect();
        let outputs = idxs.iter().map(|x| self.outputs[*x].clone()).collect();

        DoubleGroupProximityQueryOutput {
            outputs,
            shape_idxs,
            num_ground_truth_checks: self.num_ground_truth_checks,
        }
    }
}


macro_rules! create_double_group_query {
    ($func_name: ident, $output_type: ty, $query_func_code: expr, $push_code: expr, $early_stop_code: expr, $extra_args: ty) => {

        pub fn $func_name(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupProximityQueryMode, skips: Option<&DMatrix<bool>>, early_stop: bool, extra_args: $extra_args) -> DoubleGroupProximityQueryOutput<$output_type> {
            assert_eq!(group_a.len(), poses_a.len());
            assert_eq!(group_b.len(), poses_b.len());

            // let mut out = vec![];
            let mut outputs = vec![];
            let mut shape_idxs = vec![];
            let mut num_ground_truth_checks = 0;

            let mut f = |i: usize, sa: &OffsetShape, pa: &ISE3q, j: usize, sb: &OffsetShape, pb: &ISE3q| -> i8 {
                if skips.is_some() { if skips.as_ref().unwrap()[(i,j)] { return 0; } }

                let ppa = sa.get_transform(pa);
                let ppb = sb.get_transform(pb);

                let res = $query_func_code(&*ppa, &**sa.shape(), &*ppb, &**sb.shape(), &extra_args);
                if $push_code(&res) {
                    // out.push( ((i,j), res) );
                    outputs.push(res);
                    shape_idxs.push((i,j));
                }

                if early_stop {
                    if $early_stop_code(&res) { return -1; }
                }

                0
            };

            match query_mode {
                DoubleGroupProximityQueryMode::AllPossiblePairs => {
                    for (i, (sa, pa)) in group_a.iter().zip(poses_a).enumerate() {
                        for (j, (sb, pb)) in group_b.iter().zip(poses_b).enumerate() {
                            let res = f(i, sa, pa, j, sb, pb);
                            num_ground_truth_checks += 1;
                            if res == -1 { return DoubleGroupProximityQueryOutput::new(outputs, shape_idxs, num_ground_truth_checks); }
                        }
                    }
                },
                DoubleGroupProximityQueryMode::SkipSymmetricalPairs => {
                    for (i, (sa, pa)) in group_a.iter().zip(poses_a).enumerate() {
                        'l: for (j, (sb, pb)) in group_b.iter().zip(poses_b).enumerate() {
                            if i >= j { continue 'l; }
                            let res = f(i, sa, pa, j, sb, pb);
                            num_ground_truth_checks += 1;
                            if res == -1 { return DoubleGroupProximityQueryOutput::new(outputs, shape_idxs, num_ground_truth_checks); }
                        }
                    }
                },
                DoubleGroupProximityQueryMode::SubsetOfPairs(v) => {
                    for (i,j) in v {
                        let pa = &poses_a[*i];
                        let pb = &poses_b[*j];
                        let sa = &group_a[*i];
                        let sb = &group_b[*j];

                        let res = f(*i, sa, pa, *j, sb, pb);
                        num_ground_truth_checks += 1;
                        if res == -1 { return DoubleGroupProximityQueryOutput::new(outputs, shape_idxs, num_ground_truth_checks); }
                    }
                }
            }

            DoubleGroupProximityQueryOutput::new(outputs, shape_idxs, num_ground_truth_checks)
        }
    };
}

create_double_group_query!(
    pairwise_group_query_intersection,
    bool,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, _extra_args: &()|
        {
            intersection_test(&pose_a.0, shape_a, &pose_b.0, shape_b).expect("error")
        },
    |res: &bool|
        {
            return if *res { true } else { false }
        },
    |res: &bool |
        {
            return if *res { true } else { false }
        },
    ()
);

create_double_group_query!(
    pairwise_group_query_distance,
    f64,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, _extra_args: &()|
        {
            distance(&pose_a.0, shape_a, &pose_b.0, shape_b).expect("error")
        },
    |_res: &f64|
        {
            return true;
        },
    |res: &f64 |
        {
            return if *res <= 0.0 { true } else { false }
        },
    ()
);

create_double_group_query!(
    pairwise_group_query_contact,
    Option<Contact>,
    |pose_a: &ISE3q, shape_a: &dyn Shape, pose_b: &ISE3q, shape_b: &dyn Shape, extra_args: &f64|
        {
            contact(&pose_a.0, shape_a, &pose_b.0, shape_b, *extra_args).expect("error")
        },
    |_res: &Option<Contact> |
        {
            return true;
        },
    |res: &Option<Contact> |
        {
            if let Some(c) = res {
                if c.dist <= 0.0 { return true; }
            }
            return false;
        },
    f64
);


/*
pub trait ToAverageDistancesF64 {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<((usize, usize), f64)>;
}
impl ToAverageDistancesF64 for Vec<((usize, usize), f64)> {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<((usize, usize), f64)> {
        let mut out = vec![];

        self.iter().for_each(|((i,j ), d)| {
            let average = average_distances[(*i, *j)].max(0.0001);
            out.push( ((*i, *j), *d / average) )
        });

        out
    }
}

pub trait ToAverageDistancesContactOption {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<((usize, usize), Option<Contact>)>;
}
impl ToAverageDistancesContactOption for Vec<((usize, usize), Option<Contact>)> {
    fn to_average_distances(&self, average_distances: &DMatrix<f64>) -> Vec<((usize, usize), Option<Contact>)> {
        let mut out = vec![];

        self.iter().for_each(|((i, j), c)| {
            match c {
                None => { out.push( ((*i, *j), None) ) }
                Some(c) => {
                    let average = average_distances[(*i, *j)].max(0.0001);
                    let out_c = Contact {
                        point1: c.point1,
                        point2: c.point2,
                        normal1: c.normal1,
                        normal2: c.normal2,
                        dist: c.dist / average,
                    };
                    out.push(((*i, *j), Some(out_c)));
                }
            }
        });

        out
    }
}
*/
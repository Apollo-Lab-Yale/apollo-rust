
use nalgebra::DMatrix;
use parry3d_f64::query::{Contact, contact, distance, intersection_test};
use parry3d_f64::shape::Shape;
use apollo_rust_spatial::lie::se3_implicit_quaternion::ISE3q;
use crate::offset_shape::OffsetShape;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DoubleGroupQueryMode {
    AllPossiblePairs,
    SkipSymmetricalPairs,
    SubsetOfPairs(Vec<(usize, usize)>)
}

macro_rules! create_double_group_query {
    ($func_name: ident, $output_type: ty, $query_func_code: expr, $push_code: expr, $early_stop_code: expr, $extra_args: ty) => {

        pub fn $func_name(group_a: &Vec<OffsetShape>, poses_a: &Vec<ISE3q>, group_b: &Vec<OffsetShape>, poses_b: &Vec<ISE3q>, query_mode: &DoubleGroupQueryMode, skips: Option<&DMatrix<bool>>, early_stop: bool, extra_args: $extra_args) -> Vec<((usize, usize), $output_type)> {
            assert_eq!(group_a.len(), poses_a.len());
            assert_eq!(group_b.len(), poses_b.len());

            let mut out = vec![];

            let mut f = |i: usize, sa: &OffsetShape, pa: &ISE3q, j: usize, sb: &OffsetShape, pb: &ISE3q| -> i8 {
                if skips.is_some() { if skips.as_ref().unwrap()[(i,j)] { return 0; } }

                let ppa = sa.get_transform(pa);
                let ppb = sa.get_transform(pb);

                let res = $query_func_code(&*ppa, &**sa.shape(), &*ppb, &**sb.shape(), &extra_args);
                if $push_code(&res) {
                    out.push( ((i,j), res) );
                }

                if early_stop {
                    if $early_stop_code(&res) { return -1; }
                }

                0
            };

            match query_mode {
                DoubleGroupQueryMode::AllPossiblePairs => {
                    for (i, (sa, pa)) in group_a.iter().zip(poses_a).enumerate() {
                        for (j, (sb, pb)) in group_b.iter().zip(poses_b).enumerate() {
                            let res = f(i, sa, pa, j, sb, pb);
                            if res == -1 { return out; }
                        }
                    }
                },
                DoubleGroupQueryMode::SkipSymmetricalPairs => {
                    for (i, (sa, pa)) in group_a.iter().zip(poses_a).enumerate() {
                        'l: for (j, (sb, pb)) in group_b.iter().zip(poses_b).enumerate() {
                            if i >= j { continue 'l; }
                            let res = f(i, sa, pa, j, sb, pb);
                            if res == -1 { return out; }
                        }
                    }
                },
                DoubleGroupQueryMode::SubsetOfPairs(v) => {
                    for (i,j) in v {
                        let pa = &poses_a[*i];
                        let pb = &poses_b[*j];
                        let sa = &group_a[*i];
                        let sb = &group_b[*j];

                        let res = f(*i, sa, pa, *j, sb, pb);
                        if res == -1 { return out; }
                    }
                }
            }

            out
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
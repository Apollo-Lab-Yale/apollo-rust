use nalgebra::{DMatrix, DVector};
use rand::Rng;

pub type V = DVector<f64>;

pub trait ApolloDVectorTrait {
    fn new(slice: &[f64]) -> Self;
    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self;
}
impl ApolloDVectorTrait for V {
    fn new(slice: &[f64]) -> Self {
        DVector::from_column_slice(slice)
    }
    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self {
        let mut rng = rand::thread_rng();
        let mut v = DVector::zeros(n);

        for i in 0..n {
            v[i] = rng.gen_range(min..=max);
        }

        v
    }
}

pub type M = DMatrix<f64>;

pub trait ApolloDMatrixTrait {
    /// Row major slice
    fn new(slice: &[f64], nrows: usize, ncols: usize) -> Self;
    fn new_square(slice: &[f64], dim: usize) -> Self;
    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self;
    fn from_column_vectors(columns: &[V]) -> Self;
    fn from_row_vectors(rows: &[V]) -> Self;
    fn gram_schmidt_on_columns(&self) -> Self;
    fn get_all_columns(&self) -> Vec<V>;
    fn get_all_rows(&self) -> Vec<V>;
    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult;
    fn fundamental_subspaces(&self) -> FundamentalSubspaces {
        let svd = self.singular_value_decomposition(SVDType::Full);
        svd.to_fundamental_subspaces()
    }
}
impl ApolloDMatrixTrait for M {
    fn new(slice: &[f64], nrows: usize, ncols: usize) -> Self {
        DMatrix::from_row_slice(nrows, ncols, slice)
    }
    fn new_square(slice: &[f64], dim: usize) -> Self {
        Self::new(slice, dim, dim)
    }
    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self {
        let mut rng = rand::thread_rng();
        let mut m = DMatrix::zeros(nrows, ncols);

        for i in 0..nrows {
            for j in 0..ncols {
                m[(i, j)] = rng.gen_range(min..=max);
            }
        }

        m
    }
    fn from_column_vectors(columns: &[V]) -> Self {
        let mut out = M::zeros(columns[0].len(), columns.len());
        columns.iter().enumerate().for_each(|(i, x)| out.set_column(i, x));

        out
    }
    fn from_row_vectors(rows: &[V]) -> Self {
        let mut out = M::zeros(rows.len(), rows[0].len());
        rows.iter().enumerate().for_each(|(i, x)| out.set_row(i, &x.transpose()));

        out
    }
    fn gram_schmidt_on_columns(&self) -> Self {
        let columns = self.get_all_columns();
        let res = columns.gram_schmidt_process();
        return Self::from_column_vectors(&res);
    }
    fn get_all_columns(&self) -> Vec<V> {
        let mut out = vec![];
        let ncols = self.ncols();
        for i in 0..ncols {
            out.push(V::new(self.column(i).as_slice()));
        }
        out
    }
    fn get_all_rows(&self) -> Vec<V> {
        let transpose = self.transpose();
        return transpose.get_all_columns();
    }
    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult {
        let svd = self.clone().svd(true, true);
        let u = svd.u.expect("error");
        let vt = svd.v_t.expect("error");
        let rank = self.rank(0.0001);

       return match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns();
                let v = vt.transpose();
                let mut v_cols = v.get_all_columns();

                for _ in 0..m-rank {
                    u_cols.push(V::new_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process();

                for _ in 0..n-rank {
                    v_cols.push(V::new_random_with_range(n, -1.0, 1.0));
                }
                v_cols = v_cols.gram_schmidt_process();

                let full_u = M::from_column_vectors(&u_cols);
                let full_v = M::from_column_vectors(&v_cols);
                let full_vt = full_v.transpose();

                let mut sigma = Self::zeros(self.nrows(), self.ncols());
                let mut singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() { sigma[(i,i)] = *s; }

                for _ in 0..(m.max(n) - singular_values.len()) { singular_values.push(0.0); }

                SVDResult {
                    u: full_u,
                    sigma,
                    vt: full_vt,
                    singular_values,
                    rank,
                    svd_type,
                }
            }
            SVDType::Compact => {
                let mut sigma = Self::zeros(rank, rank);
                let singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() { sigma[(i,i)] = *s; }

                SVDResult {
                    u,
                    sigma,
                    vt,
                    singular_values,
                    rank,
                    svd_type,
                }
            }
        }
    }
}

pub trait ApolloMultipleDVectorsTrait {
    fn gram_schmidt_process(&self) -> Vec<V>;
}
impl ApolloMultipleDVectorsTrait for &[V] {
    fn gram_schmidt_process(&self) -> Vec<V> {
        let mut out: Vec<V> = Vec::new();

        for v in self.iter() {
            let mut u = v.clone();

            for basis_vector in &out {
                let proj = basis_vector.dot(v) / basis_vector.dot(basis_vector);
                u -= &(basis_vector * proj);
            }

            if u.norm() > 1e-10 {
                let n = u.norm();
                out.push(u / n);
            }
        }

        out
    }
}
impl ApolloMultipleDVectorsTrait for &mut Vec<V> {
    fn gram_schmidt_process(&self) -> Vec<V> {
        (**self).gram_schmidt_process()
    }
}
impl ApolloMultipleDVectorsTrait for Vec<V> {
    fn gram_schmidt_process(&self) -> Vec<V> {
        self.as_slice().gram_schmidt_process()
    }
}

#[derive(Clone, Debug)]
pub struct SVDResult {
    u: M,
    sigma: M,
    vt: M,
    singular_values: Vec<f64>,
    rank: usize,
    svd_type: SVDType
}
impl SVDResult {
    pub fn u(&self) -> &M {
        &self.u
    }
    pub fn sigma(&self) -> &M {
        &self.sigma
    }
    pub fn vt(&self) -> &M {
        &self.vt
    }
    pub fn singular_values(&self) -> &Vec<f64> {
        &self.singular_values
    }
    pub fn svd_type(&self) -> &SVDType {
        &self.svd_type
    }
    pub fn rank(&self) -> usize {
        self.rank
    }
    pub fn to_fundamental_subspaces(&self) -> FundamentalSubspaces {
        assert_eq!(self.svd_type, SVDType::Full, "svd type must be Full in order to get fundamental subspaces");

        let u_cols = self.u.get_all_columns();
        let v_cols = self.vt.get_all_rows();
        let rank = self.rank;

        let mut u1_cols = vec![];
        let mut u2_cols = vec![];
        let mut v1_cols = vec![];
        let mut v2_cols = vec![];

        for (i, col) in u_cols.iter().enumerate() {
            if i < rank { u1_cols.push(col.clone()) }
            else { u2_cols.push(col.clone()) }
        }

        for (i, col) in v_cols.iter().enumerate() {
            if i < rank { v1_cols.push(col.clone()) }
            else { v2_cols.push(col.clone()) }
        }

        FundamentalSubspaces {
            column_space_basis: if u1_cols.len() > 0 { Some(M::from_column_vectors(&u1_cols)) } else { None },
            left_null_space_basis: if u2_cols.len() > 0 { Some(M::from_column_vectors(&u2_cols)) } else { None },
            row_space_basis: if v1_cols.len() > 0 { Some(M::from_column_vectors(&v1_cols)) } else { None },
            null_space_basis: if v2_cols.len() > 0 { Some(M::from_column_vectors(&v2_cols)) } else { None },
        }
    }
}

#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum SVDType {
    Full, Compact
}

#[derive(Clone, Debug)]
pub struct FundamentalSubspaces {
    /// U_1
    column_space_basis: Option<M>,
    /// U_2
    left_null_space_basis: Option<M>,
    /// V_1
    row_space_basis: Option<M>,
    /// V_2
    null_space_basis: Option<M>
}
impl FundamentalSubspaces {
    pub fn column_space_basis(&self) -> &Option<M> {
        &self.column_space_basis
    }
    pub fn left_null_space_basis(&self) -> &Option<M> {
        &self.left_null_space_basis
    }
    pub fn row_space_basis(&self) -> &Option<M> {
        &self.row_space_basis
    }
    pub fn null_space_basis(&self) -> &Option<M> {
        &self.null_space_basis
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
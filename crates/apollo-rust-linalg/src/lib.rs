#[cfg(feature = "ad")]
pub mod linalg_ad;

extern crate core;

use std::fmt::Debug;
use nalgebra::{DMatrix, DVector};
use nalgebra_lapack::{LU, SVD};
use rand::Rng;

/// Type alias for a dynamic vector of `f64`.
pub type V = DVector<f64>;

/// Trait defining operations for `DVector`.
pub trait ApolloDVectorTrait {
    /// Creates a new vector from a slice.
    fn new(slice: &[f64]) -> Self;

    /// Creates a new vector with random values within the specified range.
    ///
    /// # Arguments
    /// * `n` - The size of the vector.
    /// * `min` - The minimum value in the range.
    /// * `max` - The maximum value in the range.
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

////////////////////////////////////////////////////////////////////////////////////////////////////

/// Type alias for a dynamic matrix of `f64`.
pub type M = DMatrix<f64>;

/// Trait defining operations for `DMatrix`.
pub trait ApolloDMatrixTrait {
    /// Creates a new matrix from a row-major slice.
    fn new(slice: &[f64], nrows: usize, ncols: usize) -> Self;

    /// Creates a new square matrix from a slice.
    fn new_square(slice: &[f64], dim: usize) -> Self;

    /// Creates a new matrix with random values within the specified range.
    ///
    /// # Arguments
    /// * `nrows` - The number of rows in the matrix.
    /// * `ncols` - The number of columns in the matrix.
    /// * `min` - The minimum value in the range.
    /// * `max` - The maximum value in the range.
    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self;

    /// Constructs a matrix from column vectors.
    fn from_column_vectors(columns: &[V]) -> Self;

    /// Constructs a matrix from row vectors.
    fn from_row_vectors(rows: &[V]) -> Self;

    /// Sets a column from the given slice
    fn set_column_from_slice(&mut self, column_idx: usize, slice: &[f64]);

    /// Sets a row from the given slice
    fn set_row_from_slice(&mut self, row_idx: usize, slice: &[f64]);

    /// Applies the Gram-Schmidt process to the columns of the matrix.
    fn gram_schmidt_on_columns(&self) -> Self;

    /// Retrieves all columns of the matrix as vectors.
    fn get_all_columns(&self) -> Vec<V>;

    /// Retrieves all rows of the matrix as vectors.
    fn get_all_rows(&self) -> Vec<V>;

    fn get_column(&self, i: usize) -> V;

    fn get_row(&self, i: usize) -> V;

    /// Computes the Singular Value Decomposition (SVD) of the matrix.
    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult;

    /// Convenience method to get the fundamental subspaces of the matrix.
    fn fundamental_subspaces(&self) -> FundamentalSubspaces {
        let svd = self.singular_value_decomposition(SVDType::Full);
        svd.to_fundamental_subspaces()
    }

    /// Computes the full QR factorization of the matrix using the Gram-Schmidt process.
    fn full_qr_factorization(&self) -> QRResult;

    fn inverse_blas(&self) -> Option<M>;

    fn singular_value_decomposition_blas(&self, svd_type: SVDType) -> SVDResult;

    fn hstack(&self, other: &DMatrix<f64>) -> Option<DMatrix<f64>>;

    fn vstack(&self, other: &DMatrix<f64>) -> Option<DMatrix<f64>>;
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

    fn set_column_from_slice(&mut self, column_idx: usize, slice: &[f64]) {
        self.set_column(column_idx, &V::from_column_slice(slice))
    }

    fn set_row_from_slice(&mut self, row_idx: usize, slice: &[f64]) {
        self.set_row(row_idx, &V::from_row_slice(slice).transpose())
    }

    fn gram_schmidt_on_columns(&self) -> Self {
        let columns = self.get_all_columns();
        let res = columns.gram_schmidt_process();
        Self::from_column_vectors(&res)
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
        transpose.get_all_columns()
    }

    fn get_column(&self, i: usize) -> V {
        return V::new(self.column(i).as_slice());
    }

    fn get_row(&self, i: usize) -> V {
        return V::new(self.row(i).transpose().as_slice());
    }

    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult {
        let svd = self.clone().svd(true, true);
        let u = svd.u.expect("error");
        let vt = svd.v_t.expect("error");
        let rank = self.rank(0.0001);

        match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns();
                let v = vt.transpose();
                let mut v_cols = v.get_all_columns();

                for _ in 0..m - rank {
                    u_cols.push(V::new_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process();

                for _ in 0..n - rank {
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
                for (i, s) in singular_values.iter().enumerate() {
                    if singular_values[i] == 0.0 { continue; }
                    sigma[(i,i)] = *s;
                }

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

    fn full_qr_factorization(&self) -> QRResult {
        let m = self.nrows();
        let n = self.ncols();
        let min_dim = m.min(n);

        let a_cols = self.get_all_columns();

        let mut q_cols: Vec<V> = Vec::with_capacity(m);
        let mut r = M::zeros(m, n);

        for j in 0..min_dim {
            let mut v = a_cols[j].clone();

            for i in 0..j {
                let q_i = &q_cols[i];
                let r_ij = q_i.dot(&a_cols[j]);
                r[(i, j)] = r_ij;
                v -= q_i * r_ij;
            }

            let v_norm = v.norm();
            if v_norm > 1e-10 {
                r[(j, j)] = v_norm;
                v /= v_norm;
            }

            q_cols.push(v);
        }

        if n > m {
            for j in m..n {
                for i in 0..m {
                    let q_i = &q_cols[i];
                    let r_ij = q_i.dot(&a_cols[j]);
                    r[(i, j)] = r_ij;
                }
            }
        }

        for _ in min_dim..m {
            let mut v = V::new_random_with_range(m, -1.0, 1.0);

            for q_i in &q_cols {
                let proj = q_i.dot(&v);
                v -= q_i * proj;
            }

            let v_norm = v.norm();
            if v_norm > 1e-10 {
                v /= v_norm;
                q_cols.push(v);
            }
        }

        let q = M::from_column_vectors(&q_cols);

        QRResult { q, r }
    }

    fn inverse_blas(&self) -> Option<M> {
        return LU::new(self.clone()).inverse();
    }

    fn singular_value_decomposition_blas(&self, svd_type: SVDType) -> SVDResult {
        let svd = SVD::new(self.clone()).expect("error");
        let u = svd.u;
        let vt = svd.vt;
        let rank = self.rank(0.0001);

        match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns();
                let v = vt.transpose();
                let mut v_cols = v.get_all_columns();

                for _ in 0..m - rank {
                    u_cols.push(V::new_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process();

                for _ in 0..n - rank {
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
                for (i, s) in singular_values.iter().enumerate() {
                    if singular_values[i] == 0.0 { continue; }
                    sigma[(i,i)] = *s;
                }

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

    fn hstack(&self, other: &DMatrix<f64>) -> Option<DMatrix<f64>> {
        if self.nrows() != other.nrows() {
            return None;
        }

        let mut result = DMatrix::zeros(self.nrows(), self.ncols() + other.ncols());
        result.view_mut((0, 0), (self.nrows(), self.ncols())).copy_from(self);
        result.view_mut((0, self.ncols()), (other.nrows(), other.ncols())).copy_from(other);

        Some(result)
    }

    fn vstack(&self, other: &DMatrix<f64>) -> Option<DMatrix<f64>> {
        if self.ncols() != other.ncols() {
            return None;
        }

        let mut result = DMatrix::zeros(self.nrows() + other.nrows(), self.ncols());
        result.view_mut((0, 0), (self.nrows(), self.ncols())).copy_from(self);
        result.view_mut((self.nrows(), 0), (other.nrows(), other.ncols())).copy_from(other);

        Some(result)
    }
}

/// Trait defining operations for a collection of `DVector`.
pub trait ApolloMultipleDVectorsTrait {
    /// Applies the Gram-Schmidt process to a collection of vectors.
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

/// Implementations of `ApolloMultipleDVectorsTrait` for mutable and owned vectors.
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

/// Struct to hold the results of Singular Value Decomposition.
#[derive(Clone, Debug)]
pub struct SVDResult {
    u: M,
    sigma: M,
    vt: M,
    singular_values: Vec<f64>,
    rank: usize,
    svd_type: SVDType,
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

#[derive(Clone, Debug)]
pub struct QRResult {
    /// Q matrix with orthonormal columns
    pub q: M,
    /// Upper triangular R matrix
    pub r: M
}

/// Enum to specify the type of Singular Value Decomposition.
#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum SVDType {
    Full,
    Compact,
}

/// Struct to hold the fundamental subspaces resulting from SVD.
#[derive(Clone, Debug)]
pub struct FundamentalSubspaces {
    /// Basis for column space.
    column_space_basis: Option<M>,
    /// Basis for left null space.
    left_null_space_basis: Option<M>,
    /// Basis for row space.
    row_space_basis: Option<M>,
    /// Basis for null space.
    null_space_basis: Option<M>,
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

/// Function to create a `DMatrix` from a 2D vector.
pub fn dmatrix_from_2dvec<T: Clone + PartialEq + Debug + 'static>(v: &Vec<Vec<T>>) -> DMatrix<T> {
    let v = v.clone();
    let rows = v.len();
    let cols = v[0].len();
    let data: Vec<T> = v.into_iter().flatten().collect();

    DMatrix::from_vec(rows, cols, data)
}

/// Function to convert a `DMatrix` to a 2D vector.
pub fn dmatrix_to_2dvec<T: Clone>(matrix: &DMatrix<T>) -> Vec<Vec<T>> {
    let (rows, cols) = matrix.shape();
    let mut vec_2d = Vec::with_capacity(rows);

    for i in 0..rows {
        let mut row_vec = Vec::with_capacity(cols);
        for j in 0..cols {
            row_vec.push(matrix[(i, j)].clone());
        }
        vec_2d.push(row_vec);
    }

    vec_2d
}

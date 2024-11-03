use ad_trait::AD;
use nalgebra::{DMatrix, DVector};
use crate::{ApolloDMatrixTrait, ApolloDVectorTrait, M, SVDType, V};

pub type VAD<A> = DVector<A>;

pub trait ApolloDVectorADTrait<A: AD> {
    fn new_ad(slice: &[A]) -> Self;

    fn new_ad_from_f64s(slice: &[f64]) -> Self;

    /// Creates a new vector with random values within the specified range.
    ///
    /// # Arguments
    /// * `n` - The size of the vector.
    /// * `min` - The minimum value in the range.
    /// * `max` - The maximum value in the range.
    fn new_ad_random_with_range(n: usize, min: f64, max: f64) -> Self;
}
impl<A: AD> ApolloDVectorADTrait<A> for DVector<A> {
    fn new_ad(slice: &[A]) -> Self {
        DVector::from_column_slice(slice)
    }

    fn new_ad_from_f64s(slice: &[f64]) -> Self {
        let v: Vec<A> = slice.iter().map(|x| A::constant(*x)).collect();
        Self::new_ad(&v)
    }

    fn new_ad_random_with_range(n: usize, min: f64, max: f64) -> Self {
        let v = V::new_random_with_range(n, min, max);
        return Self::new_ad_from_f64s(v.as_slice())
    }
}

pub type MAD<A> = DMatrix<A>;

pub trait ApolloDMatrixADTrait<A: AD> {
    fn new_ad(slice: &[A], nrows: usize, ncols: usize) -> Self;
    fn new_ad_from_f64s(slice: &[f64], nrows: usize, ncols: usize) -> Self;
    fn new_ad_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self;
    fn from_column_vectors_ad(columns: &[VAD<A>]) -> Self;
    fn from_row_vectors_ad(rows: &[VAD<A>]) -> Self;
    fn set_column_from_slice_ad(&mut self, column_idx: usize, slice: &[A]);
    fn set_row_from_slice_ad(&mut self, row_idx: usize, slice: &[A]);
    fn gram_schmidt_on_columns_ad(&self) -> Self;
    fn get_all_columns_ad(&self) -> Vec<VAD<A>>;
    fn get_all_rows_ad(&self) -> Vec<VAD<A>>;
    fn singular_value_decomposition_ad(&self, svd_type: SVDType) -> SVDResultAD<A>;
    fn fundamental_subspaces_ad(&self) -> FundamentalSubspacesAD<A> {
        let svd = self.singular_value_decomposition_ad(SVDType::Full);
        svd.to_fundamental_subspaces()
    }
    fn full_qr_factorization_ad(&self) -> QRResultAD<A>;
}
impl<A: AD> ApolloDMatrixADTrait<A> for MAD<A> {
    fn new_ad(slice: &[A], nrows: usize, ncols: usize) -> Self {
        Self::from_row_slice(nrows, ncols, slice)
    }

    fn new_ad_from_f64s(slice: &[f64], nrows: usize, ncols: usize) -> Self {
        let v: Vec<A> = slice.iter().map(|x| A::constant(*x)).collect();
        Self::new_ad(&v, nrows, ncols)
    }

    fn new_ad_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self {
        let m = M::new_random_with_range(nrows, ncols, min, max);
        Self::new_ad_from_f64s(m.as_slice(), nrows, ncols)
    }

    fn from_column_vectors_ad(columns: &[VAD<A>]) -> Self {
        let mut out = MAD::zeros(columns[0].len(), columns.len());
        columns.iter().enumerate().for_each(|(i, x)| out.set_column(i, x));
        out
    }

    fn from_row_vectors_ad(rows: &[VAD<A>]) -> Self {
        let mut out = MAD::zeros(rows.len(), rows[0].len());
        rows.iter().enumerate().for_each(|(i, x)| out.set_row(i, &x.transpose()));
        out
    }

    fn set_column_from_slice_ad(&mut self, column_idx: usize, slice: &[A]) {
        self.set_column(column_idx, &VAD::from_column_slice(slice))
    }

    fn set_row_from_slice_ad(&mut self, row_idx: usize, slice: &[A]) {
        self.set_row(row_idx, &VAD::from_row_slice(slice).transpose())
    }

    fn gram_schmidt_on_columns_ad(&self) -> Self {
        let columns = self.get_all_columns_ad();
        let res = columns.gram_schmidt_process_ad();
        Self::from_column_vectors_ad(&res)
    }

    fn get_all_columns_ad(&self) -> Vec<VAD<A>> {
        let mut out = vec![];
        let ncols = self.ncols();
        for i in 0..ncols {
            out.push(VAD::new_ad(self.column(i).as_slice()));
        }
        out
    }

    fn get_all_rows_ad(&self) -> Vec<VAD<A>> {
        let transpose = self.transpose();
        transpose.get_all_columns_ad()
    }

    fn singular_value_decomposition_ad(&self, svd_type: SVDType) -> SVDResultAD<A> {
        let svd = self.clone().svd(true, true);
        let u = svd.u.expect("error");
        let vt = svd.v_t.expect("error");
        let rank = self.rank(0.0001.into());

        match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns_ad();
                let v = vt.transpose();
                let mut v_cols = v.get_all_columns_ad();

                for _ in 0..m - rank {
                    u_cols.push(VAD::new_ad_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process_ad();

                for _ in 0..n - rank {
                    v_cols.push(VAD::new_ad_random_with_range(n, -1.0, 1.0));
                }
                v_cols = v_cols.gram_schmidt_process_ad();

                let full_u = MAD::from_column_vectors_ad(&u_cols);
                let full_v = MAD::from_column_vectors_ad(&v_cols);
                let full_vt = full_v.transpose();

                let mut sigma = MAD::zeros(self.nrows(), self.ncols());
                let mut singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() {
                    sigma[(i,i)] = s.clone();
                }

                for _ in 0..(m.max(n) - singular_values.len()) {
                    singular_values.push(A::constant(0.0));
                }

                SVDResultAD {
                    u: full_u,
                    sigma,
                    vt: full_vt,
                    singular_values,
                    rank,
                    svd_type,
                }
            }
            SVDType::Compact => {
                let mut sigma = MAD::zeros(rank, rank);
                let singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() {
                    if *s == A::constant(0.0) { continue; }
                    sigma[(i,i)] = s.clone();
                }

                SVDResultAD {
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

    fn full_qr_factorization_ad(&self) -> QRResultAD<A> {
        let m = self.nrows();
        let n = self.ncols();
        let min_dim = m.min(n);

        let a_cols = self.get_all_columns_ad();

        let mut q_cols: Vec<VAD<A>> = Vec::with_capacity(m);
        let mut r = MAD::zeros(m, n);

        for j in 0..min_dim {
            let mut v = a_cols[j].clone();

            for i in 0..j {
                let q_i = &q_cols[i];
                let r_ij = q_i.dot(&a_cols[j]);
                r[(i, j)] = r_ij;
                v -= q_i * r_ij;
            }

            let v_norm = v.norm();
            if v_norm > A::constant(1e-10) {
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
            let mut v = VAD::new_ad_random_with_range(m, -1.0, 1.0);

            for q_i in &q_cols {
                let proj = q_i.dot(&v);
                v -= q_i * proj;
            }

            let v_norm = v.norm();
            if v_norm > A::constant(1e-10) {
                v /= v_norm;
                q_cols.push(v);
            }
        }

        let q = MAD::from_column_vectors_ad(&q_cols);

        QRResultAD { q, r }
    }
}

/// Trait defining operations for a collection of `DVector` with autodiff-compatible elements.
pub trait ApolloMultipleDVectorsADTrait<A: AD> {
    /// Applies the Gram-Schmidt process to a collection of vectors.
    fn gram_schmidt_process_ad(&self) -> Vec<VAD<A>>;
}
impl<A: AD> ApolloMultipleDVectorsADTrait<A> for &[VAD<A>] {
    fn gram_schmidt_process_ad(&self) -> Vec<VAD<A>> {
        let mut out: Vec<VAD<A>> = Vec::new();

        for v in self.iter() {
            let mut u = v.clone();

            for basis_vector in &out {
                let proj = basis_vector.dot(v) / basis_vector.dot(basis_vector);
                u -= &(basis_vector * proj);
            }

            if u.norm() > A::constant(1e-10) {
                let n = u.norm();
                out.push(u / n);
            }
        }

        out
    }
}
impl<A: AD> ApolloMultipleDVectorsADTrait<A> for Vec<VAD<A>> {
    fn gram_schmidt_process_ad(&self) -> Vec<VAD<A>> {
        self.as_slice().gram_schmidt_process_ad()
    }
}

#[derive(Clone, Debug)]
pub struct SVDResultAD<A: AD> {
    u: MAD<A>,
    sigma: MAD<A>,
    vt: MAD<A>,
    singular_values: Vec<A>,
    rank: usize,
    svd_type: SVDType,
}
impl<A: AD> SVDResultAD<A> {
    pub fn u(&self) -> &MAD<A> {
        &self.u
    }
    pub fn sigma(&self) -> &MAD<A> {
        &self.sigma
    }
    pub fn vt(&self) -> &MAD<A> {
        &self.vt
    }
    pub fn singular_values(&self) -> &Vec<A> {
        &self.singular_values
    }
    pub fn svd_type(&self) -> &SVDType {
        &self.svd_type
    }
    pub fn rank(&self) -> usize {
        self.rank
    }
    pub fn to_fundamental_subspaces(&self) -> FundamentalSubspacesAD<A> {
        assert_eq!(self.svd_type, SVDType::Full, "SVD type must be Full to get fundamental subspaces");

        let u_cols = self.u.get_all_columns_ad();
        let v_cols = self.vt.get_all_rows_ad();
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

        FundamentalSubspacesAD {
            column_space_basis: if u1_cols.len() > 0 { Some(MAD::from_column_vectors_ad(&u1_cols)) } else { None },
            left_null_space_basis: if u2_cols.len() > 0 { Some(MAD::from_column_vectors_ad(&u2_cols)) } else { None },
            row_space_basis: if v1_cols.len() > 0 { Some(MAD::from_column_vectors_ad(&v1_cols)) } else { None },
            null_space_basis: if v2_cols.len() > 0 { Some(MAD::from_column_vectors_ad(&v2_cols)) } else { None },
        }
    }
}

#[derive(Clone, Debug)]
pub struct QRResultAD<A: AD> {
    /// Q matrix with orthonormal columns
    pub q: MAD<A>,
    /// Upper triangular R matrix
    pub r: MAD<A>,
}

#[derive(Clone, Debug)]
pub struct FundamentalSubspacesAD<A: AD> {
    /// Basis for column space.
    column_space_basis: Option<MAD<A>>,
    /// Basis for left null space.
    left_null_space_basis: Option<MAD<A>>,
    /// Basis for row space.
    row_space_basis: Option<MAD<A>>,
    /// Basis for null space.
    null_space_basis: Option<MAD<A>>,
}
impl<A: AD> FundamentalSubspacesAD<A> {
    pub fn column_space_basis(&self) -> &Option<MAD<A>> {
        &self.column_space_basis
    }
    pub fn left_null_space_basis(&self) -> &Option<MAD<A>> {
        &self.left_null_space_basis
    }
    pub fn row_space_basis(&self) -> &Option<MAD<A>> {
        &self.row_space_basis
    }
    pub fn null_space_basis(&self) -> &Option<MAD<A>> {
        &self.null_space_basis
    }
}
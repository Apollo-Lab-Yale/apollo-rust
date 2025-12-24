use ad_trait::AD;
use nalgebra::{Const, DMatrix, DVector, Dyn, Matrix, VecStorage};

pub type V<A> = DVector<A>;

pub trait ApolloDVectorTrait<A: AD> {
    fn new(slice: &[A]) -> Self;

    fn new_from_f64s(slice: &[f64]) -> Self;

    /// Creates a new vector with random values within the specified range.
    ///
    /// # Arguments
    /// * `n` - The size of the vector.
    /// * `min` - The minimum value in the range.
    /// * `max` - The maximum value in the range.
    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self;

    fn to_other_ad_type<A2: AD>(&self) -> V<A2>;

    fn to_constant_ad(&self) -> V<A>;
}
impl<A: AD> ApolloDVectorTrait<A> for DVector<A> {
    fn new(slice: &[A]) -> Self {
        DVector::from_column_slice(slice)
    }

    fn new_from_f64s(slice: &[f64]) -> Self {
        let v: Vec<A> = slice.iter().map(|x| A::constant(*x)).collect();
        Self::new(&v)
    }

    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self {
        let v = <Matrix<f64, Dyn, Const<1>, VecStorage<f64, Dyn, Const<1>>> as apollo_rust_linalg::ApolloDVectorTrait>::new_random_with_range(n, min, max);
        return Self::new_from_f64s(v.as_slice());
    }

    fn to_other_ad_type<A2: AD>(&self) -> V<A2> {
        let s = self
            .as_slice()
            .iter()
            .map(|x| x.to_other_ad_type::<A2>())
            .collect::<Vec<A2>>();
        return V::new(&s);
    }

    fn to_constant_ad(&self) -> V<A> {
        let s = self
            .as_slice()
            .iter()
            .map(|x| x.to_constant_ad())
            .collect::<Vec<A>>();
        return V::new(&s);
    }
}

pub type M<A> = DMatrix<A>;

pub trait ApolloDMatrixTrait<A: AD> {
    fn new(slice: &[A], nrows: usize, ncols: usize) -> Self;
    fn new_from_f64s(slice: &[f64], nrows: usize, ncols: usize) -> Self;
    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self;
    fn from_column_vectors(columns: &[V<A>]) -> Self;
    fn from_row_vectors(rows: &[V<A>]) -> Self;
    fn set_column_from_slice(&mut self, column_idx: usize, slice: &[A]);
    fn set_row_from_slice(&mut self, row_idx: usize, slice: &[A]);
    fn gram_schmidt_on_columns(&self) -> Self;
    fn get_all_columns(&self) -> Vec<V<A>>;
    fn get_all_rows(&self) -> Vec<V<A>>;
    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult<A>;
    fn fundamental_subspaces(&self) -> FundamentalSubspaces<A> {
        let svd = self.singular_value_decomposition(SVDType::Full);
        svd.to_fundamental_subspaces()
    }
    fn full_qr_factorization(&self) -> QRResult<A>;
    fn to_other_ad_type<A2: AD>(&self) -> M<A2>;
    fn to_constant_ad(&self) -> M<A>;
}
impl<A: AD> ApolloDMatrixTrait<A> for M<A> {
    fn new(slice: &[A], nrows: usize, ncols: usize) -> Self {
        Self::from_row_slice(nrows, ncols, slice)
    }

    fn new_from_f64s(slice: &[f64], nrows: usize, ncols: usize) -> Self {
        let v: Vec<A> = slice.iter().map(|x| A::constant(*x)).collect();
        Self::new(&v, nrows, ncols)
    }

    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self {
        let m = <Matrix<f64, Dyn, Dyn, VecStorage<f64, Dyn, Dyn>> as apollo_rust_linalg::ApolloDMatrixTrait>::new_random_with_range(nrows, ncols, min, max);
        Self::new_from_f64s(m.as_slice(), nrows, ncols)
    }

    fn from_column_vectors(columns: &[V<A>]) -> Self {
        let mut out = M::zeros(columns[0].len(), columns.len());
        columns
            .iter()
            .enumerate()
            .for_each(|(i, x)| out.set_column(i, x));
        out
    }

    fn from_row_vectors(rows: &[V<A>]) -> Self {
        let mut out = M::zeros(rows.len(), rows[0].len());
        rows.iter()
            .enumerate()
            .for_each(|(i, x)| out.set_row(i, &x.transpose()));
        out
    }

    fn set_column_from_slice(&mut self, column_idx: usize, slice: &[A]) {
        self.set_column(column_idx, &V::from_column_slice(slice))
    }

    fn set_row_from_slice(&mut self, row_idx: usize, slice: &[A]) {
        self.set_row(row_idx, &V::from_row_slice(slice).transpose())
    }

    fn gram_schmidt_on_columns(&self) -> Self {
        let columns = self.get_all_columns();
        let res = columns.gram_schmidt_process();
        Self::from_column_vectors(&res)
    }

    fn get_all_columns(&self) -> Vec<V<A>> {
        let mut out = vec![];
        let ncols = self.ncols();
        for i in 0..ncols {
            out.push(<DVector<A>>::new(self.column(i).as_slice()));
        }
        out
    }

    fn get_all_rows(&self) -> Vec<V<A>> {
        let transpose = self.transpose();
        transpose.get_all_columns()
    }

    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult<A> {
        let svd = self.clone().svd(true, true);
        let u = svd.u.expect("error");
        let vt = svd.v_t.expect("error");
        let rank = self.rank(0.0001.into());

        match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns();
                let v = vt.transpose();
                let mut v_cols = v.get_all_columns();

                for _ in 0..m - rank {
                    u_cols.push(<DVector<A>>::new_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process();

                for _ in 0..n - rank {
                    v_cols.push(<DVector<A>>::new_random_with_range(n, -1.0, 1.0));
                }
                v_cols = v_cols.gram_schmidt_process();

                let full_u = <DMatrix<A>>::from_column_vectors(&u_cols);
                let full_v = <DMatrix<A>>::from_column_vectors(&v_cols);
                let full_vt = full_v.transpose();

                let mut sigma = M::zeros(self.nrows(), self.ncols());
                let mut singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() {
                    sigma[(i, i)] = s.clone();
                }

                for _ in 0..(m.max(n) - singular_values.len()) {
                    singular_values.push(A::constant(0.0));
                }

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
                let mut sigma = M::zeros(rank, rank);
                let singular_values = svd.singular_values.data.as_vec().clone();
                for (i, s) in singular_values.iter().enumerate() {
                    if *s == A::constant(0.0) {
                        continue;
                    }
                    sigma[(i, i)] = s.clone();
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

    fn full_qr_factorization(&self) -> QRResult<A> {
        let m = self.nrows();
        let n = self.ncols();
        let min_dim = m.min(n);

        let a_cols = self.get_all_columns();

        let mut q_cols: Vec<V<A>> = Vec::with_capacity(m);
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
            let mut v = <DVector<A>>::new_random_with_range(m, -1.0, 1.0);

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

        let q = <DMatrix<A>>::from_column_vectors(&q_cols);

        QRResult { q, r }
    }

    fn to_other_ad_type<A2: AD>(&self) -> M<A2> {
        let (nrows, ncols) = self.shape();
        let s = self
            .as_slice()
            .iter()
            .map(|x| x.to_other_ad_type::<A2>())
            .collect::<Vec<A2>>();
        return M::new(&s, nrows, ncols);
    }

    fn to_constant_ad(&self) -> M<A> {
        let (nrows, ncols) = self.shape();
        let s = self
            .as_slice()
            .iter()
            .map(|x| x.to_constant_ad())
            .collect::<Vec<A>>();
        return M::new(&s, nrows, ncols);
    }
}

/// Trait defining operations for a collection of `DVector` with autodiff-compatible elements.
pub trait ApolloMultipleDVectorsTrait<A: AD> {
    /// Applies the Gram-Schmidt process to a collection of vectors.
    fn gram_schmidt_process(&self) -> Vec<V<A>>;
}
impl<A: AD> ApolloMultipleDVectorsTrait<A> for &[V<A>] {
    fn gram_schmidt_process(&self) -> Vec<V<A>> {
        let mut out: Vec<V<A>> = Vec::new();

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
impl<A: AD> ApolloMultipleDVectorsTrait<A> for Vec<V<A>> {
    fn gram_schmidt_process(&self) -> Vec<V<A>> {
        self.as_slice().gram_schmidt_process()
    }
}

/// Enum to specify the type of Singular Value Decomposition.
#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum SVDType {
    Full,
    Compact,
}

#[derive(Clone, Debug)]
pub struct SVDResult<A: AD> {
    u: M<A>,
    sigma: M<A>,
    vt: M<A>,
    singular_values: Vec<A>,
    rank: usize,
    svd_type: SVDType,
}
impl<A: AD> SVDResult<A> {
    pub fn u(&self) -> &M<A> {
        &self.u
    }
    pub fn sigma(&self) -> &M<A> {
        &self.sigma
    }
    pub fn vt(&self) -> &M<A> {
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
    pub fn to_fundamental_subspaces(&self) -> FundamentalSubspaces<A> {
        assert_eq!(
            self.svd_type,
            SVDType::Full,
            "SVD type must be Full to get fundamental subspaces"
        );

        let u_cols = self.u.get_all_columns();
        let v_cols = self.vt.get_all_rows();
        let rank = self.rank;

        let mut u1_cols = vec![];
        let mut u2_cols = vec![];
        let mut v1_cols = vec![];
        let mut v2_cols = vec![];

        for (i, col) in u_cols.iter().enumerate() {
            if i < rank {
                u1_cols.push(col.clone())
            } else {
                u2_cols.push(col.clone())
            }
        }

        for (i, col) in v_cols.iter().enumerate() {
            if i < rank {
                v1_cols.push(col.clone())
            } else {
                v2_cols.push(col.clone())
            }
        }

        FundamentalSubspaces {
            column_space_basis: if u1_cols.len() > 0 {
                Some(<DMatrix<A>>::from_column_vectors(&u1_cols))
            } else {
                None
            },
            left_null_space_basis: if u2_cols.len() > 0 {
                Some(<DMatrix<A>>::from_column_vectors(&u2_cols))
            } else {
                None
            },
            row_space_basis: if v1_cols.len() > 0 {
                Some(<DMatrix<A>>::from_column_vectors(&v1_cols))
            } else {
                None
            },
            null_space_basis: if v2_cols.len() > 0 {
                Some(<DMatrix<A>>::from_column_vectors(&v2_cols))
            } else {
                None
            },
        }
    }
}

#[derive(Clone, Debug)]
pub struct QRResult<A: AD> {
    /// Q matrix with orthonormal columns
    pub q: M<A>,
    /// Upper triangular R matrix
    pub r: M<A>,
}

#[derive(Clone, Debug)]
pub struct FundamentalSubspaces<A: AD> {
    /// Basis for column space.
    column_space_basis: Option<M<A>>,
    /// Basis for left null space.
    left_null_space_basis: Option<M<A>>,
    /// Basis for row space.
    row_space_basis: Option<M<A>>,
    /// Basis for null space.
    null_space_basis: Option<M<A>>,
}
impl<A: AD> FundamentalSubspaces<A> {
    pub fn column_space_basis(&self) -> &Option<M<A>> {
        &self.column_space_basis
    }
    pub fn left_null_space_basis(&self) -> &Option<M<A>> {
        &self.left_null_space_basis
    }
    pub fn row_space_basis(&self) -> &Option<M<A>> {
        &self.row_space_basis
    }
    pub fn null_space_basis(&self) -> &Option<M<A>> {
        &self.null_space_basis
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vector_new() {
        let values = [1.0, 2.0, 3.0];
        let v = <V<f64>>::new_from_f64s(&values);
        assert_eq!(v.len(), 3);
        assert_eq!(v[0], 1.0);
        assert_eq!(v[2], 3.0);
    }

    #[test]
    fn test_vector_random() {
        let n = 10;
        let v = <V<f64>>::new_random_with_range(n, -1.0, 1.0);
        assert_eq!(v.len(), n);
        for i in 0..n {
            assert!(v[i] >= -1.0 && v[i] <= 1.0);
        }
    }

    #[test]
    fn test_matrix_new() {
        let m = <M<f64>>::new_from_f64s(&[1.0, 2.0, 3.0, 4.0], 2, 2);
        assert_eq!(m.nrows(), 2);
        assert_eq!(m.ncols(), 2);
        assert_eq!(m[(0, 0)], 1.0);
        assert_eq!(m[(0, 1)], 2.0);
    }

    #[test]
    fn test_matrix_from_vectors() {
        let v1 = <V<f64>>::new_from_f64s(&[1.0, 2.0]);
        let v2 = <V<f64>>::new_from_f64s(&[3.0, 4.0]);

        let m_col = <M<f64>>::from_column_vectors(&[v1.clone(), v2.clone()]);
        assert_eq!(m_col[(0, 0)], 1.0);
        assert_eq!(m_col[(0, 1)], 3.0);

        let m_row = <M<f64>>::from_row_vectors(&[v1, v2]);
        assert_eq!(m_row[(0, 0)], 1.0);
        assert_eq!(m_row[(0, 1)], 2.0);
    }

    #[test]
    fn test_gram_schmidt() {
        let v1 = <V<f64>>::new_from_f64s(&[1.0, 1.0, 0.0]);
        let v2 = <V<f64>>::new_from_f64s(&[1.0, 0.0, 1.0]);
        let v3 = <V<f64>>::new_from_f64s(&[0.0, 1.0, 1.0]);

        let input = vec![v1, v2, v3];
        let orthogonal = input.gram_schmidt_process();

        assert_eq!(orthogonal.len(), 3);

        for i in 0..3 {
            assert!((orthogonal[i].norm() - 1.0).abs() < 1e-10);
            for j in 0..i {
                assert!(orthogonal[i].dot(&orthogonal[j]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_qr_factorization() {
        let a = <M<f64>>::new_from_f64s(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 3, 2);
        let qr = a.full_qr_factorization();

        let reconstructed = &qr.q * &qr.r;
        for i in 0..a.nrows() {
            for j in 0..a.ncols() {
                assert!((a[(i, j)] - reconstructed[(i, j)]).abs() < 1e-10);
            }
        }

        let qtq = qr.q.transpose() * &qr.q;
        let identity = <M<f64>>::identity(qr.q.ncols(), qr.q.ncols());
        for i in 0..identity.nrows() {
            for j in 0..identity.ncols() {
                assert!((qtq[(i, j)] - identity[(i, j)]).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_svd_full() {
        let a = <M<f64>>::new_from_f64s(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0], 3, 2);
        let svd = a.singular_value_decomposition(SVDType::Full);

        let reconstructed = &svd.u * &svd.sigma * &svd.vt;
        for i in 0..a.nrows() {
            for j in 0..a.ncols() {
                assert!((a[(i, j)] - reconstructed[(i, j)]).abs() < 1e-10);
            }
        }

        assert_eq!(svd.rank(), 2);
    }

    #[test]
    fn test_fundamental_subspaces() {
        let a = <M<f64>>::new_from_f64s(&[1.0, 0.0, 0.0, 1.0, 0.0, 0.0], 3, 2);
        let svd = a.singular_value_decomposition(SVDType::Full);
        let subspaces = svd.to_fundamental_subspaces();

        assert!(subspaces.column_space_basis().is_some());
        assert!(subspaces.left_null_space_basis().is_some());
        assert!(subspaces.row_space_basis().is_some());
        assert!(subspaces.null_space_basis().is_none());
    }
}

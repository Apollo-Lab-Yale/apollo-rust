use std::fmt::Debug;
use nalgebra::{DMatrix, DVector};
use rand::Rng;

// Type alias for a dynamic vector of f64
pub type V = DVector<f64>;

// Trait defining operations for DVector
pub trait ApolloDVectorTrait {
    // Creates a new vector from a slice
    fn new(slice: &[f64]) -> Self;
    // Creates a new vector with random values in a specified range
    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self;
}

// Implementation of ApolloDVectorTrait for DVector<f64>
impl ApolloDVectorTrait for V {
    fn new(slice: &[f64]) -> Self {
        DVector::from_column_slice(slice)
    }

    fn new_random_with_range(n: usize, min: f64, max: f64) -> Self {
        let mut rng = rand::thread_rng(); // Initialize a random number generator
        let mut v = DVector::zeros(n);    // Create a zero-initialized vector of length n

        for i in 0..n {
            v[i] = rng.gen_range(min..=max); // Fill the vector with random values
        }

        v
    }
}

// Type alias for a dynamic matrix of f64
pub type M = DMatrix<f64>;

// Trait defining operations for DMatrix
pub trait ApolloDMatrixTrait {
    /// Creates a new matrix from a row-major slice
    fn new(slice: &[f64], nrows: usize, ncols: usize) -> Self;
    // Creates a new square matrix from a slice
    fn new_square(slice: &[f64], dim: usize) -> Self;
    // Creates a new matrix with random values in a specified range
    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self;
    // Constructs a matrix from column vectors
    fn from_column_vectors(columns: &[V]) -> Self;
    // Constructs a matrix from row vectors
    fn from_row_vectors(rows: &[V]) -> Self;
    // Applies the Gram-Schmidt process to the columns
    fn gram_schmidt_on_columns(&self) -> Self;
    // Retrieves all columns of the matrix as vectors
    fn get_all_columns(&self) -> Vec<V>;
    // Retrieves all rows of the matrix as vectors
    fn get_all_rows(&self) -> Vec<V>;
    // Computes the Singular Value Decomposition (SVD) of the matrix
    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult;
    // Convenience method to get fundamental subspaces
    fn fundamental_subspaces(&self) -> FundamentalSubspaces {
        let svd = self.singular_value_decomposition(SVDType::Full);
        svd.to_fundamental_subspaces()
    }
}

// Implementation of ApolloDMatrixTrait for DMatrix<f64>
impl ApolloDMatrixTrait for M {
    fn new(slice: &[f64], nrows: usize, ncols: usize) -> Self {
        DMatrix::from_row_slice(nrows, ncols, slice)
    }

    fn new_square(slice: &[f64], dim: usize) -> Self {
        Self::new(slice, dim, dim)
    }

    fn new_random_with_range(nrows: usize, ncols: usize, min: f64, max: f64) -> Self {
        let mut rng = rand::thread_rng(); // Initialize a random number generator
        let mut m = DMatrix::zeros(nrows, ncols); // Create a zero-initialized matrix

        for i in 0..nrows {
            for j in 0..ncols {
                m[(i, j)] = rng.gen_range(min..=max); // Fill the matrix with random values
            }
        }

        m
    }

    fn from_column_vectors(columns: &[V]) -> Self {
        let mut out = M::zeros(columns[0].len(), columns.len()); // Initialize a zero matrix
        columns.iter().enumerate().for_each(|(i, x)| out.set_column(i, x)); // Set columns

        out
    }

    fn from_row_vectors(rows: &[V]) -> Self {
        let mut out = M::zeros(rows.len(), rows[0].len()); // Initialize a zero matrix
        rows.iter().enumerate().for_each(|(i, x)| out.set_row(i, &x.transpose())); // Set rows

        out
    }

    fn gram_schmidt_on_columns(&self) -> Self {
        let columns = self.get_all_columns(); // Get all columns as vectors
        let res = columns.gram_schmidt_process(); // Apply Gram-Schmidt process
        Self::from_column_vectors(&res) // Reconstruct matrix from orthogonal columns
    }

    fn get_all_columns(&self) -> Vec<V> {
        let mut out = vec![];
        let ncols = self.ncols();
        for i in 0..ncols {
            out.push(V::new(self.column(i).as_slice())); // Extract each column as a vector
        }
        out
    }

    fn get_all_rows(&self) -> Vec<V> {
        let transpose = self.transpose(); // Transpose the matrix
        transpose.get_all_columns() // Get rows by extracting columns from transposed matrix
    }

    fn singular_value_decomposition(&self, svd_type: SVDType) -> SVDResult {
        let svd = self.clone().svd(true, true); // Perform SVD on a clone of the matrix
        let u = svd.u.expect("error"); // Extract U matrix
        let vt = svd.v_t.expect("error"); // Extract V^T matrix
        let rank = self.rank(0.0001); // Compute the rank of the matrix

        match svd_type {
            SVDType::Full => {
                let m = self.nrows();
                let n = self.ncols();

                let mut u_cols = u.get_all_columns(); // Get columns of U
                let v = vt.transpose(); // Transpose V^T to get V
                let mut v_cols = v.get_all_columns(); // Get columns of V

                // Add random columns to U to make it full rank
                for _ in 0..m - rank {
                    u_cols.push(V::new_random_with_range(m, -1.0, 1.0));
                }
                u_cols = u_cols.gram_schmidt_process(); // Orthogonalize columns of U

                // Add random columns to V to make it full rank
                for _ in 0..n - rank {
                    v_cols.push(V::new_random_with_range(n, -1.0, 1.0));
                }
                v_cols = v_cols.gram_schmidt_process(); // Orthogonalize columns of V

                let full_u = M::from_column_vectors(&u_cols); // Reconstruct full U matrix
                let full_v = M::from_column_vectors(&v_cols); // Reconstruct full V matrix
                let full_vt = full_v.transpose(); // Reconstruct full V^T matrix

                let mut sigma = Self::zeros(self.nrows(), self.ncols()); // Initialize Sigma matrix
                let mut singular_values = svd.singular_values.data.as_vec().clone(); // Extract singular values
                for (i, s) in singular_values.iter().enumerate() { sigma[(i,i)] = *s; } // Fill Sigma matrix

                // Pad singular values with zeros if necessary
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
                let mut sigma = Self::zeros(rank, rank); // Initialize Sigma matrix for compact SVD
                let singular_values = svd.singular_values.data.as_vec().clone(); // Extract singular values
                for (i, s) in singular_values.iter().enumerate() {
                    if singular_values[i] == 0.0 { continue; } // Skip zero singular values
                    sigma[(i,i)] = *s; // Fill Sigma matrix
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
}

// Trait defining operations for a collection of DVector
pub trait ApolloMultipleDVectorsTrait {
    // Applies Gram-Schmidt process to a collection of vectors
    fn gram_schmidt_process(&self) -> Vec<V>;
}

// Implementation of ApolloMultipleDVectorsTrait for a slice of DVector
impl ApolloMultipleDVectorsTrait for &[V] {
    fn gram_schmidt_process(&self) -> Vec<V> {
        let mut out: Vec<V> = Vec::new();

        for v in self.iter() {
            let mut u = v.clone();

            // Orthogonalize vector against previously processed vectors
            for basis_vector in &out {
                let proj = basis_vector.dot(v) / basis_vector.dot(basis_vector);
                u -= &(basis_vector * proj);
            }

            // Normalize and add to output if it's not nearly zero
            if u.norm() > 1e-10 {
                let n = u.norm();
                out.push(u / n);
            }
        }

        out
    }
}

// Implementations of ApolloMultipleDVectorsTrait for mutable and owned vectors
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

// Struct to hold results of Singular Value Decomposition
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

        let u_cols = self.u.get_all_columns(); // Get columns of U matrix
        let v_cols = self.vt.get_all_rows(); // Get rows of V^T matrix
        let rank = self.rank;

        let mut u1_cols = vec![];
        let mut u2_cols = vec![];
        let mut v1_cols = vec![];
        let mut v2_cols = vec![];

        // Separate columns into those corresponding to the rank and those not
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

// Enum to specify the type of Singular Value Decomposition
#[derive(Clone, Debug, Copy, PartialEq, Eq)]
pub enum SVDType {
    Full, Compact
}

// Struct to hold the fundamental subspaces resulting from SVD
#[derive(Clone, Debug)]
pub struct FundamentalSubspaces {
    /// Basis for column space
    column_space_basis: Option<M>,
    /// Basis for left null space
    left_null_space_basis: Option<M>,
    /// Basis for row space
    row_space_basis: Option<M>,
    /// Basis for null space
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

// Function to create a DMatrix from a 2D Vec
pub fn dmatrix_from_2dvec<T: Clone + PartialEq + Debug + 'static>(v: &Vec<Vec<T>>) -> DMatrix<T> {
    let v = v.clone(); // Clone the input vector

    let rows = v.len();
    let cols = v[0].len();

    let data: Vec<T> = v.into_iter().flatten().collect(); // Flatten the 2D vector into a 1D vector

    DMatrix::from_vec(rows, cols, data) // Create a DMatrix from the flattened data
}

// Function to convert a DMatrix to a 2D Vec
pub fn dmatrix_to_2dvec<T: Clone>(matrix: &DMatrix<T>) -> Vec<Vec<T>> {
    let (rows, cols) = matrix.shape(); // Get matrix dimensions
    let mut vec_2d = Vec::with_capacity(rows); // Initialize 2D vector

    for i in 0..rows {
        let mut row_vec = Vec::with_capacity(cols); // Initialize row vector
        for j in 0..cols {
            row_vec.push(matrix[(i, j)].clone()); // Collect each element in the row vector
        }
        vec_2d.push(row_vec); // Add row vector to the 2D vector
    }

    vec_2d
}

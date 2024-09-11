// Generate the power set of a given vector. The power set is the set of all subsets of the vector.
pub fn power_set<T: Clone>(vec: Vec<T>) -> Vec<Vec<T>> {
    let n = vec.len(); // Get the length of the input vector.
    let mut result = Vec::new(); // Initialize a vector to store all subsets.

    // Iterate over all possible combinations of elements (2^n combinations).
    for i in 0..(1 << n) {
        let mut subset = Vec::new(); // Initialize a vector to store the current subset.
        // Check each bit of the current combination.
        for j in 0..n {
            if i & (1 << j) != 0 { // If the j-th bit is set in i, include vec[j] in the subset.
                subset.push(vec[j].clone());
            }
        }
        result.push(subset); // Add the current subset to the result.
    }

    result // Return the power set.
}

// Generate all combinations of `n` elements from a given vector.
pub fn combinations_of_n<T: Clone>(vec: Vec<T>, n: usize) -> Vec<Vec<T>> {
    let mut result = Vec::new(); // Initialize a vector to store combinations.
    let len = vec.len(); // Get the length of the input vector.

    if n > len { // If n is greater than the length of the vector, no combinations are possible.
        return result;
    }

    // Initialize the first combination of indices.
    let mut indices = (0..n).collect::<Vec<_>>();

    loop {
        // Generate a combination from the current indices and add it to the result.
        result.push(indices.iter().map(|&i| vec[i].clone()).collect());

        let mut i = n;
        // Find the rightmost index that can be incremented.
        while i > 0 {
            i -= 1;
            if indices[i] != i + len - n {
                break;
            }
        }

        // If no more combinations are possible, break the loop.
        if i == 0 && indices[i] == len - n {
            break;
        }

        // Increment the found index and adjust subsequent indices.
        indices[i] += 1;
        for j in i+1..n {
            indices[j] = indices[j-1] + 1;
        }
    }

    result // Return the vector of combinations.
}

// Trait to convert a vector of `Option<T>` into a vector of vectors of `T`.
pub trait VecOfOptionsToVecOfVecsTrait<T> where T: Clone {
    fn to_vec_of_vecs(&self) -> Vec<Vec<T>>;
}

// Implementation of the trait for vectors of `Option<T>`.
impl<T> VecOfOptionsToVecOfVecsTrait<T> for Vec<Option<T>> where T: Clone {
    fn to_vec_of_vecs(&self) -> Vec<Vec<T>> {
        // Convert each `Option<T>` into a vector of `T`, with `None` becoming an empty vector.
        self.iter().map(|x| {
            match x {
                None => { vec![] } // Convert `None` to an empty vector.
                Some(x) => { vec![x.clone()] } // Convert `Some(x)` to a vector containing `x`.
            }
        }).collect() // Collect the results into a vector of vectors.
    }
}

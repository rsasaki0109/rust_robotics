//! # KD-Tree based Nearest neighbor search in Rust
//!
//! ## Example
//!
//! ```
//! use nearest_neighbor::KdTree;
//! use nalgebra::SVector;
//!
//! let raw_data: [[f64; 2]; 11] = [
//!     [-3., -3.],   // 0
//!     [-3., -1.],   // 1
//!     [-3., 3.],    // 2
//!     [-1., -1.],   // 3
//!     [-1., 3.],    // 4
//!     [1., 3.],     // 5
//!     [2., -3.],    // 6
//!     [2., -1.],    // 7
//!     [2., 3.],     // 8
//!     [3., -1.],    // 9
//!     [4., 1.],     // 10
//! ];
//!
//! let data: Vec<SVector<f64, 2>> = raw_data.iter().map(|&e| e.into()).collect();
//! let leaf_size = 2;
//!
//! let tree = KdTree::new(&data, leaf_size);
//!
//! let query = SVector::<f64, 2>::new(0., -4.);
//!
//! // Searches the nearest element from the query in the target data.
//! let (argmin, distance) = tree.search(&query);
//! assert_eq!(argmin, Some(6));  // Sixth element [2, -3] in `data` is the nearest
//! assert_eq!(distance, 5.);     // Squared euclidean distance from (0, -4) to (2, -3)
//! ```

#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(not(feature = "std"))]
use log::info; // Use log crate when building application

#[cfg(feature = "std")]
use std::println as info;

mod vecmap;

#[macro_use]
extern crate alloc;

use crate::alloc::string::ToString;
use crate::vecmap::VecMap;
use alloc::collections::BTreeMap;
use alloc::vec::Vec;
use core::cmp::Ordering;
use core::fmt::{Debug, Display};
use core::ops::{AddAssign, MulAssign, SubAssign};
use num_traits::float::FloatCore;

use nalgebra::SVector;

pub trait Float: 'static + FloatCore + Debug + Display + SubAssign + AddAssign + MulAssign {}
impl<T: 'static + FloatCore + Debug + Display + SubAssign + AddAssign + MulAssign> Float for T {}

fn divide<T: Float, const D: usize>(
    indices: &mut Vec<usize>,
    data: &[SVector<T, D>],
    dim: usize,
) -> (T, Vec<usize>, Vec<usize>) {
    let cmp = |i1: &usize, i2: &usize| -> Ordering {
        let d1: &SVector<T, D> = &data[*i1];
        let d2: &SVector<T, D> = &data[*i2];
        d1[dim].partial_cmp(&d2[dim]).unwrap()
    };

    indices.sort_unstable_by(cmp);

    let mut k = indices.len() / 2;
    while k > 0 && data[indices[k]][dim] == data[indices[k - 1]][dim] {
        k = k - 1;
    }
    let indices_r = indices.split_off(k);
    let indices_l = indices.clone();
    (data[indices_r[0]][dim], indices_l, indices_r)
}

#[inline]
fn panic_leaf_node_not_found<T: Float, const D: usize>(
    query: &SVector<T, D>,
    leaf_index: usize,
) -> ! {
    panic!(
        "Leaf node corresponding to query = {:?} node_index = {} not found. \
        There's something wrong in the tree construction. \
        Report the bug to the repository owner.",
        query, leaf_index
    )
}

#[inline]
fn calc_depth(node_index: usize) -> usize {
    assert!(node_index > 0);
    let mut i = 2;
    let mut depth = 0;
    while i <= node_index {
        i *= 2;
        depth += 1;
    }
    depth
}

#[inline]
fn find_dim<const D: usize>(node_index: usize) -> usize {
    assert!(node_index > 0);
    calc_depth(node_index) % D
}

#[inline]
fn squared_euclidean<T: Float, const D: usize>(a: &SVector<T, D>, b: &SVector<T, D>) -> T {
    let d = a - b;
    d.dot(&d)
}

#[inline]
fn squared_diff<T: Float>(a: T, b: T) -> T {
    (a - b) * (a - b)
}

#[inline]
fn distance_to_boundary<T: Float, const D: usize>(
    query: &SVector<T, D>,
    boundary: T,
    dim: usize,
) -> T {
    squared_diff(query[(dim, 0)], boundary)
}

#[inline]
fn children_near_far<T: Float>(query_element: T, boundary: T, node_index: usize) -> (usize, usize) {
    if query_element < boundary {
        (node_index * 2 + 0, node_index * 2 + 1)
    } else {
        (node_index * 2 + 1, node_index * 2 + 0)
    }
}

#[inline]
fn the_other_side_index(node_index: usize) -> usize {
    if node_index % 2 == 0 {
        node_index + 1
    } else {
        node_index - 1
    }
}

#[inline]
fn find_nearest<T: Float, const D: usize>(
    query: &SVector<T, D>,
    indices: &[usize],
    data: &[SVector<T, D>],
) -> (Option<usize>, T) {
    let mut min_distance = T::infinity();
    let mut argmin = None;
    for &index in indices {
        let d = squared_euclidean(query, &data[index]);
        if d < min_distance {
            min_distance = d;
            argmin = Some(index);
        }
    }
    (argmin, min_distance)
}

#[inline]
fn find_leaf<T: Float, const D: usize>(query: &SVector<T, D>, boundaries: &VecMap<T>) -> usize {
    let mut node_index = 1;
    let mut dim: usize = 0;
    while let Some(&boundary) = boundaries.get(&node_index) {
        node_index = if query[(dim, 0)] < boundary {
            node_index * 2 + 0
        } else {
            node_index * 2 + 1
        };
        dim = (dim + 1) % D;
    }
    node_index
}

fn print_tree<T: Float, const D: usize>(
    boundaries: &VecMap<T>,
    leaves: &BTreeMap<usize, Vec<usize>>,
    data: &[SVector<T, D>],
) {
    let mut stack = Vec::from([(1, 0)]);
    while stack.len() != 0 {
        let (node_index, dim) = stack.pop().unwrap();

        let depth = calc_depth(node_index);
        if let Some(indices) = leaves.get(&node_index) {
            info!(
                "{} {:3}  {:?}",
                " ".repeat(2 * depth),
                node_index,
                indices
                    .iter()
                    .map(|&i| data[i])
                    .collect::<Vec<SVector<T, D>>>()
            );
            continue;
        };

        let b = match boundaries.get(&node_index) {
            None => "".to_string(),
            Some(boundary) => format!("{:.5}", boundary),
        };
        info!(
            "{} index = {:2}:  dim = {}:  boundary = {}",
            " ".repeat(2 * depth),
            node_index,
            dim,
            b
        );

        stack.push((node_index * 2 + 0, (dim + 1) % D));
        stack.push((node_index * 2 + 1, (dim + 1) % D));
    }
}

// Remove indices that correspond to the same data value.
//
// The data and their indices below
//
// index   data
// 0       [13., 1.],
// 1       [13., 1.],
// 2       [13., 10.],
// 3       [13., 10.],
//
// must be summarized into
//
// index   data
// 0       [13., 1.],
// 2       [13., 10.],
//
//
// We need this procedure to make any data element fit in a leaf.
// If the data contains more duplicated data elements than the leaf size,
// duplicated elements cannot fit in a leaf.
// If you can certainly assume that data does not contain any duplicated
// element, you can just stop using this function and init indices as
// let indices = (0..data.len()).collect::<Vec<usize>>();
fn non_duplicate_indices<T: Float, const D: usize>(data: &[SVector<T, D>]) -> Vec<usize> {
    let cmp = |i1: &usize, i2: &usize| -> Ordering {
        for dim in 0..D {
            let d1 = &data[*i1][dim];
            let d2 = &data[*i2][dim];
            let ord = d1.partial_cmp(&d2).unwrap();
            if ord != Ordering::Equal {
                return ord;
            }
        }
        return Ordering::Equal;
    };

    let mut indices = (0..data.len()).collect::<Vec<usize>>();
    indices.sort_by(cmp);
    let cmp = |i1: &mut usize, i2: &mut usize| -> bool { data[*i1] == data[*i2] };
    indices.dedup_by(cmp);
    indices
}

/// KD-Tree for nearest neighbor search.
pub struct KdTree<'a, T: Float, const D: usize> {
    data: &'a [SVector<T, D>],
    /// Maps a node_index to a boundary value
    boundaries: VecMap<T>,
    /// Maps a node_index (must be a leaf) to data indices in the leaf
    leaves: BTreeMap<usize, Vec<usize>>,
}

impl<'a, T: Float, const D: usize> KdTree<'a, T, D> {
    fn find_within_distance(
        &self,
        node_index: usize,
        query: &SVector<T, D>,
        argmin: &Option<usize>,
        min_distance: T,
    ) -> (Option<usize>, T) {
        let mut argmin = *argmin;
        let mut min_distance = min_distance;
        let mut stack = Vec::from([(node_index, find_dim::<D>(node_index))]);
        while stack.len() != 0 {
            let (node_index, dim) = stack.pop().unwrap();
            let maybe_boundary = self.boundaries.get(&node_index);

            let Some(&boundary) = maybe_boundary else {
                // let find_nearest_time = std::time::Instant::now();
                // If `node_index` is not in boundaries, `node_index` must be a leaf.
                let indices = self.leaves.get(&node_index).unwrap();
                let (candidate, distance) = find_nearest(query, &indices, self.data);
                if distance < min_distance {
                    argmin = candidate;
                    min_distance = distance;
                }
                continue;
            };

            let (near, far) = children_near_far(query[(dim, 0)], boundary, node_index);

            let next_dim = (dim + 1) % D;
            stack.push((near, next_dim));

            // If the nearest element is closer than the boundary, we don't
            // need to search the farther side than the boundary.
            if min_distance < distance_to_boundary(query, boundary, dim) {
                continue;
            }
            stack.push((far, next_dim));
        }
        (argmin, min_distance)
    }

    fn find_nearest_in_other_areas(
        &self,
        query: &SVector<T, D>,
        argmin: &Option<usize>,
        distance: T,
        node_index: usize,
    ) -> (Option<usize>, T) {
        let mut node_index = node_index;
        let mut argmin = *argmin;
        let mut distance = distance;
        let mut boundary_dim = (find_dim::<D>(node_index) + D - 1) % D;
        while node_index > 1 {
            let the_other_side_index = the_other_side_index(node_index);
            let parent_index = node_index / 2;
            let &boundary = self.boundaries.get(&parent_index).unwrap();
            if distance > distance_to_boundary(query, boundary, boundary_dim) {
                (argmin, distance) =
                    self.find_within_distance(the_other_side_index, query, &argmin, distance);
            }
            // If we simply write `(boundary_dim - 1) % D` this will overflow
            // in case boundary_dim = 0 so we need to add D
            boundary_dim = (boundary_dim + D - 1) % D;
            node_index = parent_index;
        }
        (argmin, distance)
    }

    /// Constructs a new KD-Tree from the given slice of `nalgebra::SVector`
    /// and `leaf_size`.
    pub fn new(data: &'a [SVector<T, D>], leaf_size: usize) -> Self {
        assert!(leaf_size > 0);
        let indices = non_duplicate_indices(data);
        let mut boundaries = VecMap::<T>::new();
        let mut leaves = BTreeMap::<usize, Vec<usize>>::new();

        let mut stack = Vec::from([(indices, 1, 0)]);
        while stack.len() != 0 {
            let (mut indices, node_index, dim) = stack.pop().unwrap();

            if indices.len() <= leaf_size {
                leaves.insert(node_index, indices);
                continue;
            }

            let (boundary, indices_l, indices_r) = divide(&mut indices, data, dim);

            boundaries.insert(node_index, boundary);
            let next_dim = (dim + 1) % D;
            stack.push((indices_l, node_index * 2 + 0, next_dim));
            stack.push((indices_r, node_index * 2 + 1, next_dim));
        }
        KdTree {
            data,
            boundaries,
            leaves,
        }
    }

    pub fn print(&self) {
        print_tree::<T, D>(&self.boundaries, &self.leaves, &self.data);
    }

    /// Returns the index of the nearest item from the given query in the target data,
    /// along with the distance from the query to the nearest item.
    /// The index is None if the target data is empty.
    pub fn search(&self, query: &SVector<T, D>) -> (Option<usize>, T) {
        let leaf_index = find_leaf(query, &self.boundaries);

        let Some(indices) = self.leaves.get(&leaf_index) else {
            panic_leaf_node_not_found(query, leaf_index);
        };

        let (argmin, distance) = find_nearest(query, &indices, self.data);
        self.find_nearest_in_other_areas(query, &argmin, distance, leaf_index)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use rand::distributions::{Distribution, Uniform};

    fn to_vecs(data: &[[f64; 2]]) -> Vec<SVector<f64, 2>> {
        data.iter()
            .map(|&s| s.into())
            .collect::<Vec<SVector<f64, 2>>>()
    }

    #[test]
    fn test_divide1() {
        let raw_data: [[f64; 2]; 10] = [
            [13., 10.],
            [16., 14.],
            [14., 11.],
            [11., 18.],
            [15., 12.],
            [10., 13.],
            [17., 15.],
            [12., 19.],
            [19., 18.],
            [18., 16.],
        ];

        let dim = 0;
        let data = to_vecs(&raw_data);
        let mut indices = (0..data.len()).collect();
        let (boundary, indices_l, indices_r) = divide(&mut indices, &data, dim);
        assert_eq!(boundary, 15.);
        for &i in indices_l.iter() {
            assert!(data[i][dim] < boundary);
        }
        for &i in indices_r.iter() {
            assert!(data[i][dim] >= boundary);
        }
    }

    #[test]
    fn test_divide2() {
        let raw_data: [[f64; 2]; 11] = [
            [13., 10.],
            [16., 14.],
            [14., 11.],
            [11., 18.],
            [15., 12.],
            [10., 13.],
            [17., 15.],
            [12., 19.],
            [19., 18.],
            [18., 16.],
            [20., 16.],
        ];

        let dim = 0;
        let data = to_vecs(&raw_data);
        let mut indices = (0..data.len()).collect();
        let (boundary, indices_l, indices_r) = divide(&mut indices, &data, dim);
        assert_eq!(boundary, 15.);
        for &i in indices_l.iter() {
            assert!(data[i][dim] < boundary);
        }
        for &i in indices_r.iter() {
            assert!(data[i][dim] >= boundary);
        }
    }

    #[test]
    fn test_divide3() {
        let raw_data: [[f64; 2]; 11] = [
            [13., 10.],
            [17., 14.],
            [15., 13.],
            [15., 15.],
            [15., 19.],
            [13., 11.],
            [13., 18.],
            [13., 12.],
            [17., 18.],
            [17., 16.],
            [28., 16.],
        ];

        let dim = 0;
        let data = to_vecs(&raw_data);
        let mut indices = (0..data.len()).collect();
        let (boundary, indices_l, indices_r) = divide(&mut indices, &data, dim);
        assert_eq!(boundary, 15.);
        for &i in indices_l.iter() {
            assert!(data[i][dim] < boundary);
        }
        for &i in indices_r.iter() {
            assert!(data[i][dim] >= boundary);
        }
    }

    #[test]
    fn test_find_leaf() {
        let raw_data: [[f64; 2]; 28] = [
            [2., 2.],
            [3., 7.],
            [3., 13.],
            [3., 18.],
            [5., 10.],
            [6., 15.],
            [7., 6.],
            [8., 3.],
            [8., 18.],
            [10., 8.],
            [10., 11.],
            [10., 14.],
            [11., 4.],
            [11., 6.],
            [13., 1.],
            [13., 1.],
            [13., 10.],
            [13., 16.],
            [14., 7.],
            [14., 19.],
            [15., 4.],
            [15., 12.],
            [15., 12.],
            [15., 12.],
            [17., 17.],
            [18., 5.],
            [18., 8.],
            [18., 10.],
        ];

        let vecs = to_vecs(&raw_data);
        let tree = KdTree::new(&vecs, 2);

        let leaf_index = find_leaf(&SVector::<f64, 2>::new(10., 15.), &tree.boundaries);
        assert_eq!(leaf_index, 23);

        let leaf_index = find_leaf(&SVector::<f64, 2>::new(11., 13.), &tree.boundaries);
        assert_eq!(leaf_index, 28);

        let leaf_index = find_leaf(&SVector::<f64, 2>::new(3., 2.), &tree.boundaries);
        assert_eq!(leaf_index, 16);

        let leaf_index = find_leaf(&SVector::<f64, 2>::new(8., 17.), &tree.boundaries);
        assert_eq!(leaf_index, 23);
    }

    #[test]
    fn test_search_leaf_size_2() {
        let raw_data: [[f64; 2]; 25] = [
            [2., 2.],   // 0
            [3., 7.],   // 1
            [3., 13.],  // 2
            [3., 18.],  // 3
            [5., 10.],  // 4
            [6., 15.],  // 5
            [7., 6.],   // 6
            [8., 3.],   // 7
            [8., 18.],  // 8
            [10., 8.],  // 9
            [10., 11.], // 10
            [10., 14.], // 11
            [11., 4.],  // 12
            [11., 6.],  // 13
            [13., 1.],  // 14
            [13., 10.], // 15
            [13., 16.], // 16
            [14., 7.],  // 17
            [14., 19.], // 18
            [15., 4.],  // 19
            [15., 12.], // 20
            [17., 17.], // 21
            [18., 5.],  // 22
            [18., 8.],  // 23
            [18., 10.], // 24
        ];

        let vecs = to_vecs(&raw_data);

        let leaf_size = 2;

        let tree = KdTree::new(&vecs, leaf_size);
        for query in vecs.iter() {
            let (argmin, distance) = tree.search(query);
            assert_eq!(vecs[argmin.unwrap()], *query);
            assert_eq!(distance, 0.);
        }

        let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(10., 15.));
        assert_eq!(argmin, Some(11));
        assert_eq!(distance, 1.);

        let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(6., 3.));
        assert_eq!(argmin, Some(7));
        assert_eq!(distance, 4.);

        let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(5., 12.));
        assert_eq!(argmin, Some(4));
        assert_eq!(distance, 4.);
    }

    #[test]
    fn test_search_leaf_size_1() {
        let raw_data: [[f64; 2]; 10] = [
            [-4., 5.],  //  0
            [-3., -5.], //  1
            [-3., -3.], //  2
            [-3., 2.],  //  3
            [1., 1.],   //  4
            [1., 3.],   //  5
            [2., -2.],  //  6
            [3., 2.],   //  7
            [3., 4.],   //  8
            [5., -2.],  //  9
        ];

        let vecs = to_vecs(&raw_data);

        let leaf_size = 1;

        let tree = KdTree::new(&vecs, leaf_size);
        for query in vecs.iter() {
            let (argmin, distance) = tree.search(query);
            assert_eq!(vecs[argmin.unwrap()], *query);
            assert_eq!(distance, 0.);
        }

        let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(0., -2.));
        assert_eq!(argmin, Some(6));
        assert_eq!(distance, 4.);

        let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(-4., 1.));
        assert_eq!(argmin, Some(3));
        assert_eq!(distance, 2.);
    }

    #[test]
    fn test_non_duplicate_indices() {
        let raw_data: [[f64; 2]; 18] = [
            [3., 1.], // 0
            [3., 1.], // 1
            [4., 5.], // 2
            [3., 1.], // 3
            [3., 1.], // 4
            [2., 3.], // 5
            [3., 3.], // 6
            [3., 3.], // 7
            [1., 1.], // 8
            [1., 1.], // 9
            [1., 3.], // 10
            [1., 3.], // 11
            [2., 3.], // 12
            [2., 3.], // 13
            [2., 1.], // 14
            [2., 3.], // 15
            [3., 1.], // 16
            [4., 1.], // 17
        ];

        // After sorting
        // let raw_data: [[f64; 2]; 18] = [
        //     [1., 1.],   // 8
        //     [1., 1.],   // 9
        //     [1., 3.],   // 10
        //     [1., 3.],   // 11
        //     [2., 1.],   // 14
        //     [2., 3.],   // 5
        //     [2., 3.],   // 12
        //     [2., 3.],   // 13
        //     [2., 3.],   // 15
        //     [3., 1.],   // 0
        //     [3., 1.],   // 1
        //     [3., 1.],   // 3
        //     [3., 1.],   // 4
        //     [3., 1.],   // 16
        //     [3., 3.],   // 6
        //     [3., 3.],   // 7
        //     [4., 1.],   // 17
        //     [4., 5.],   // 2
        // ];

        // After removing duplicates
        // let raw_data: [[f64; 2]; 18] = [
        //     [1., 1.],   // 8
        //     [1., 3.],   // 10
        //     [2., 1.],   // 14
        //     [2., 3.],   // 5
        //     [3., 1.],   // 0
        //     [3., 3.],   // 6
        //     [4., 1.],   // 17
        //     [4., 5.],   // 2
        // ];

        let vecs = to_vecs(&raw_data);
        let indices = non_duplicate_indices(&vecs);
        assert_eq!(indices, Vec::<usize>::from([8, 10, 14, 5, 0, 6, 17, 2]));
    }

    #[test]
    fn test_new_from_too_few_elements() {
        // Fewer elements than the leaf size
        let raw_data: [[f64; 2]; 3] = [[-3., -1.], [-3., 3.], [-1., -1.]];
        let leaf_size = 4;
        let vecs = to_vecs(&raw_data);
        let tree = KdTree::new(&vecs, leaf_size);

        let query = SVector::<f64, 2>::new(-3., 2.);
        let (argmin, distance) = tree.search(&query);
        assert_eq!(argmin, Some(1));
        assert_eq!(distance, squared_euclidean(&query, &vecs[1]));
    }

    #[test]
    fn test_new_from_duplicated_elements() {
        // The tree building process will not stop
        // if we don't remove indices of duplicated data elements
        let raw_data: [[f64; 2]; 15] = [
            [-3., -1.], // 0
            [-3., -1.], // 1
            [-3., -1.], // 2
            [-3., -1.], // 3
            [-3., -1.], // 4
            [-3., 3.],  // 5
            [-3., -3.], // 6
            [-1., -1.], // 7
            [-1., 3.],  // 8
            [1., 3.],   // 9
            [2., -3.],  // 10
            [2., -1.],  // 11
            [2., 3.],   // 12
            [3., -1.],  // 13
            [4., 1.],   // 14
        ];
        let leaf_size = 1;
        let vecs = to_vecs(&raw_data);
        let tree = KdTree::new(&vecs, leaf_size);

        let query = SVector::<f64, 2>::new(0., -4.);
        let (argmin, distance) = tree.search(&query);
        assert_eq!(argmin, Some(10));
        assert_eq!(distance, squared_euclidean(&query, &vecs[10]));
    }

    #[test]
    fn test_find_nearest_in_other_areas() {
        let raw_data: [[f64; 2]; 11] = [
            [-3., -3.], // 0
            [-3., -1.], // 1
            [-3., 3.],  // 2
            [-1., -1.], // 3
            [-1., 3.],  // 4
            [1., 3.],   // 5
            [2., -3.],  // 6
            [2., -1.],  // 7
            [2., 3.],   // 8
            [3., -1.],  // 9
            [4., 1.],   // 10
        ];
        let leaf_size = 2;
        let vecs = to_vecs(&raw_data);
        let tree = KdTree::new(&vecs, leaf_size);

        let query = SVector::<f64, 2>::new(0., -4.);
        let (argmin, distance) = tree.search(&query);
        assert_eq!(argmin, Some(6));
        assert_eq!(distance, squared_euclidean(&query, &vecs[6]));

        let query = SVector::<f64, 2>::new(0.9, -0.9);
        let (argmin, distance) = tree.search(&query);
        assert_eq!(argmin, Some(7));
        assert_eq!(distance, squared_euclidean(&query, &vecs[7]));
    }

    #[test]
    fn test_search_on_uniform_random_f64() {
        let between = Uniform::from(-100..100);
        let mut rng = rand::thread_rng();

        const D: usize = 5;
        let mut random_uniform_vector = || {
            let mut v = SVector::<f64, D>::default();
            for i in 0..D {
                v[i] = between.sample(&mut rng) as f64;
            }
            v
        };

        let vecs = (0..5000)
            .map(|_| random_uniform_vector())
            .collect::<Vec<SVector<f64, D>>>();

        let tree = KdTree::new(&vecs, 2);
        let indices: Vec<usize> = (0..vecs.len()).collect();

        for _ in 0..100 {
            let query = random_uniform_vector();
            let (_argmin, distance) = tree.search(&query);
            let (_argmin_true, distance_true) = find_nearest(&query, &indices, &vecs);

            // There's a possibility to retrieve the item with the same
            // distance as the ground truth but in a different index so we
            // don't evaluate argmin here
            assert_eq!(distance, distance_true);
        }
    }

    #[test]
    fn test_search_on_uniform_random_f32() {
        let between = Uniform::from(-100..100);
        let mut rng = rand::thread_rng();

        const D: usize = 5;
        let mut random_uniform_vector = || {
            let mut v = SVector::<f32, D>::default();
            for i in 0..D {
                v[i] = between.sample(&mut rng) as f32;
            }
            v
        };

        let vecs = (0..5000)
            .map(|_| random_uniform_vector())
            .collect::<Vec<SVector<f32, D>>>();

        let tree = KdTree::new(&vecs, 2);
        let indices: Vec<usize> = (0..vecs.len()).collect();

        for _ in 0..100 {
            let query = random_uniform_vector();
            let (_argmin, distance) = tree.search(&query);
            let (_argmin_true, distance_true) = find_nearest(&query, &indices, &vecs);

            // There's a possibility to retrieve the item with the same
            // distance as the ground truth but in a different index so we
            // don't evaluate argmin here
            assert_eq!(distance, distance_true);
        }
    }

    #[test]
    fn test_find_dim() {
        assert_eq!(find_dim::<2>(1), 0);
        assert_eq!(find_dim::<2>(2), 1);
        assert_eq!(find_dim::<2>(3), 1);
        assert_eq!(find_dim::<2>(4), 0);
        assert_eq!(find_dim::<2>(5), 0);
        assert_eq!(find_dim::<2>(10), 1);

        assert_eq!(find_dim::<4>(1), 0);
        assert_eq!(find_dim::<4>(2), 1);
        assert_eq!(find_dim::<4>(3), 1);
        assert_eq!(find_dim::<4>(4), 2);
        assert_eq!(find_dim::<4>(5), 2);
        assert_eq!(find_dim::<4>(10), 3);
        assert_eq!(find_dim::<4>(11), 3);
        assert_eq!(find_dim::<4>(18), 0);
        assert_eq!(find_dim::<4>(52), 1);
        assert_eq!(find_dim::<4>(63), 1);
        assert_eq!(find_dim::<4>(64), 2);
    }
}

use nalgebra::SVector;
use nearest_neighbor::KdTree;

fn main() {
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

    let vecs = raw_data
        .iter()
        .map(|&s| s.into())
        .collect::<Vec<SVector<f64, 2>>>();

    let leaf_size = 2;

    let tree = KdTree::new(&vecs, leaf_size);

    let (argmin, distance) = tree.search(&SVector::<f64, 2>::new(10., 15.));

    assert_eq!(argmin, Some(11));
    assert_eq!(distance, 1.);
}

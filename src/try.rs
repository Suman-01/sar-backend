// mod geofence;
// use geofence::{local_coords, euclidian_distance};

// pub fn main() {
//     let mut cord = local_coords();
//     cord.push(local_coords()[0]);

//     // Calculate and print the Euclidean distance between each pair of consecutive points in the geofence
//     for i in 0..cord.len() - 1 {
//         let dist = euclidian_distance(cord[i], cord[i + 1]);
//         println!("Distance between {:?} and {:?} is {:.3} meters", cord[i], cord[i + 1], dist);
//     }
// }


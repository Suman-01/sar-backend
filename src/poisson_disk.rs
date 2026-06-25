use rand::{RngExt};
use rand::distr::{Distribution, Uniform};
use std::f64::consts::TAU;
use crate::geofence::{MIN_RAD, N};

// Ray-casting 
pub fn is_inside_polygon(pt: (f64, f64), corners: &Vec<(f64, f64)>) -> bool {
    let (e, n) = pt;
    let mut count = 0;

    for i in 0..corners.len() {
        let (e1, n1) = corners[i];
        let (e2, n2) = corners[(i + 1) % corners.len()];

        if (n1 > n) != (n2 > n) && e < (e2 - e1) * (n - n1)/(n2 - n1) + e1 {
            count += 1;
        }
    }
    count % 2 == 1
}

pub fn bridson_algo(corners: Vec<(f64, f64)>, r: f64, k: usize) -> Vec<(f64, f64)> {
    //initialize rng
    let mut rng = rand::rng();

    // Bounding box
    let (min_e, min_n) = corners.iter().fold((f64::MAX, f64::MAX), |(x, y), (new_x, new_y)| (x.min(*new_x), y.min(*new_y)));
    let (max_e, max_n) = corners.iter().fold((f64::MIN, f64::MIN), |(x, y), (new_x, new_y)| (x.max(*new_x), y.max(*new_y)));
    let (width, height) = (max_e - min_e, max_n - min_n);

    // Grid setup
    let cell_size = MIN_RAD / (N as f64).sqrt(); // cell size n / sqrt(r)
    let cols = (width / cell_size).ceil() as usize;
    let rows = (height / cell_size).ceil() as usize;
    let mut grid = vec![vec![None as Option<(f64, f64)>; cols]; rows];

    // get grid coordinates for a point (returns the (col, row) of the grid cell containing the point)
    fn grid_coords(pt: (f64, f64), min_e: f64, min_n: f64, cell_size: f64) -> (usize, usize){
        let e = ((pt.0 - min_e) / cell_size) as usize;
        let n = ((pt.1 - min_n) / cell_size) as usize;
        (e, n)
    }

    // Seed - Start point (using ray-casting)
    //setup distributions
    let e_dist = Uniform::new(min_e, max_e).unwrap();
    let n_dist = Uniform::new(min_n, max_n).unwrap();
    let start_pt: (f64, f64) = loop {
        let e = e_dist.sample(&mut rng);
        let n = n_dist.sample(&mut rng);

        if is_inside_polygon((e, n), &corners) {
            break (e, n);
        }
    };

    // Active list and samples
    let mut active_list = vec![start_pt];
    let mut samples = vec![start_pt];
    let (e, n ) = grid_coords(start_pt, min_e, min_n, cell_size);
    grid[n][e] = Some(start_pt);

    // angle and radius distributions
    let angle_dist = Uniform::new(0.0, TAU).unwrap();
    let radius_dist = Uniform::new(r, 2.0 * r).unwrap();

    while !active_list.is_empty() {
        let idx = rng.random_range(0..active_list.len());
        let pt = active_list[idx];
        let mut found = false;
        // search the k neighbours for a point
        for _ in 0..k {
            let angle = angle_dist.sample(&mut rng);
            let radius = radius_dist.sample(&mut rng);
            let new_pt = (pt.0 + radius * angle.cos(), pt.1 +  radius * angle.sin());

            // Check 1: Is the new point in the bounding box and inside the polygon?
            if new_pt.0 >= min_e && new_pt.0 <= max_e && new_pt.1 >= min_n && new_pt.1 <= max_n && is_inside_polygon(new_pt, &corners) {
                let (ge, gn) = grid_coords(new_pt, min_e, min_n, cell_size);

                // Check 2: Is the new point at least r away from existing points in the grid? (for a 5x5 grid)
                let mut too_close = false;
                for col in ge.saturating_sub(2)..(ge + 3).min(cols) {
                    for row in gn.saturating_sub(2)..(gn + 3).min(rows) {
                        if let Some(neighbour) = grid[row][col] {
                            let dist = ((neighbour.0 - new_pt.0).powi(2) + (neighbour.1 - new_pt.1).powi(2)).sqrt();
                            if dist < r as f64 {
                                too_close = true;
                                break;
                            }
                        }
                    }
                    if too_close {
                        break;
                    }
                }
                if !too_close {
                    grid[gn][ge] = Some(new_pt);
                    active_list.push(new_pt);
                    samples.push(new_pt);
                    found = true;
                    break;
                }
            }
        }
        if !found {
            active_list.swap_remove(idx);
        }
    }
    samples
}


// pub fn main() {
//     let corners = local_coords();
//     let seeds = bridson_algo(corners, MIN_RAD, K); 
//     println!("{:?}", seeds);
// }

// LOCAL COORDS:
// (23.0431, -14.1333)
// (-0.4452, -8.6549)
// (18.8129, 6.0322)
// (40.1863, 1.8941)

// BOUNDING BOX:
// (min_x, min_y) = (-0.4452, -14.1333)
// (max_x, max_y) = (40.1863, 6.0322)
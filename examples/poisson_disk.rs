// Poisson disk sampling
// Input: geofence corners, min_radius
// Output: list of seeds (lat/lon) that are at least min_radius apart

// Step 1: Choose a random initial seed as first seed
// Step 2: Genearate candidate seeds within min_radius and 2*min_radius
// Step 3: Check eligibility of each candidate 
// Step 4: Yes, add to seed list, No, discard
// Step 5: Repeat until no more candidates can be generated

pub fn poisson_disk_sampling(
    top_left: (f64, f64), 
    top_right: (f64, f64), 
    bottom_left: (f64, f64), 
    bottom_right: (f64, f64),
    min_radius: f64
) -> Vec<(f64, f64)> {
    // choose random initial seed
    random_seed
}

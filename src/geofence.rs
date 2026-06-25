pub const E_RADIUS: f64 = 6_378_137.0;
pub const PI: f64 = std::f64::consts::PI;
pub const CURR: (f64, f64) = (37.412101, -121.998396);
pub const MIN_RAD: f64 = 4.0;
pub const K: usize = 30; // neighbor candidates
pub const N: usize = 2; // dimension

pub fn get_coords() -> Vec<(f64, f64)> {
    vec![
        (37.412308, -121.998881), // Top left
        (37.412097, -121.998693), // Bottom left
        (37.412270, -121.998189), // Bottom right
        (37.412462, -121.998331), // Top right
    ]
}

pub fn local_coords() -> Vec<(f64, f64)> {
    let coords = get_coords();
    let mut local_geofence = Vec::new();    // Vec<(e, n)>

    for (lat, lon) in coords {
        let n = (lat - CURR.0) * (PI * E_RADIUS) / 180.0;
        let e = (lon - CURR.1) * E_RADIUS * (lat + CURR.0).to_radians().cos() * PI / 180.0;
        local_geofence.push((e, n));
    }
    local_geofence
}

// pub fn euclidian_distance(p1: (f64, f64), p2: (f64, f64)) -> f64 {
//     let (e1, n1) = p1;
//     let (e2, n2) = p2;
//     ((n2 - n1).powi(2) + (e2 - e1).powi(2)).sqrt()
// }

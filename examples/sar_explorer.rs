// src/area_traverse_gps_dynamic_split.rs
use rclrs::*;
use px4_msgs::msg::{TrajectorySetpoint, VehicleGlobalPosition};
use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

// ---------- CONFIG ----------
const GPS_CORNERS: [(f64, f64); 4] = [
    // user-provided (corrected earlier obvious typo)
    (37.412308, -121.998881),   //top left
    (37.412097, -121.998693),   //bottom left
    (37.412462, -121.998331),   //top right
    (37.412270, -121.998189),   //bottom right
];

const ALTITUDE: f32 = 5.0;     // meters (both drones same height)


fn timestamp_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_micros() as u64
}

//mpd: meters per degree
fn mpd(lat: f64) -> (f64, f64) {
    let lat_mpd = 111_133.0_f64;
    let lon_mpd = 111_133_f64 * lat_mpd.to_radians().cos(); 
    (lat_mpd, lon_mpd)
}

//WGS84 deg -> local NED meters frame (relative to lat0, lon0)
fn geo_to_local(lat: f64, lon: f64, lat0: f64, lon0: f64) -> (f32, f32) {
    let (lat_m, lon_m) = mpd(lat0); // lat lon per m
    let dx = (lat - lat0) * lat_m;  // north (x)
    let dy = (lon - lon0) * lon_m;  // east(y)
    (dx as f32, dy as f32)
}


fn gen_waypts(max_x: f32, max_y: f32, min_x: f32, min_y: 32) -> Vec<Vec<f32, f32>; 2> {
    let mut wpt_dr1 = Vec::new();
    let mut wpt_dr2: 
}
// Simple Voronoi-based parition 

// Given the corners of a geofence Global Coordinates, the job is:
// 1. Partition the geofence into Voronoi cells (no. of partions based on search efficiency for 2 drones prototype)

// min Radius = 10 (in meters)
// Euclidian distance fn to assign each point in the geofence to a seed
// out: voronoi cells (list of lat/lon points) for each seed
use crate::arm::ArmDrone;
use crate::offboard::OffboardController;
use rclrs::Node;
use std::cmp::Ordering;
use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

const GLOBAL_CORNERS: [(f64, f64); 4] = [
    (37.412308, -121.998881), // top left
    (37.412097, -121.998693), // bottom left
    (37.412462, -121.998331), // top right
    (37.412270, -121.998189), // bottom right
];
const MIN_RAD: f64 = 10.0; // in meters

pub fn poisson_disk_sampling()

// Generate seeeds based on poisson disk sampling followed by Lloyd's relaxation
pub fn create_seeds(num_seeds: i32) -> {
    // Poisson disk sampling to generate initial seeds
}

pub fn voronoi_partition() -> Result<(), Box<dyn Error>> {

}


// step 1: 
use sar_backend::mission::{lawnmower_slice_search, Waypoint};

fn main() {
    // Define a small rectangular area 
    let lat_min = 0.0;
    let lon_min = 0.0;
    let lat_max = 0.02;
    let lon_max = 0.02;

    let n_drones = 2;
    let swath_width = 100.0;   // distance between scan lines
    let altitude = 50.0;   // meters

    let missions: Vec<Vec<Waypoint>> =
        lawnmower_slice_search(lat_min, lon_min, lat_max, lon_max, n_drones, swath_width, altitude);

    // Print assigned waypoints for each drone
    // for (i, waypoints) in missions.into_iter().enumerate() {
    //     println!("--- Drone {} mission ---", i + 1);
    //     println!("  WP {} -> lat: {:.6}, lon: {:.6}", "Waypoint", "Latitude", "Longitude");
    // }

    for (i, waypoints) in missions.into_iter().enumerate() {
        let drone_id = (i + 1) as u32;
        println!("Drone {} assigned to {} waypoints.\n", drone_id, waypoints.len());
        println!("{:<10} | {:>10} | {:>10} |", "Waypoint", "Longitude", "Latitude");

        let mut cnt = 1;
        for wp in waypoints {
            println!("{:<10} | {:>10.6} | {:>10.5} |", cnt, wp.get_longitude(), wp.get_latitude());
            cnt += 1;
        }
    }
}
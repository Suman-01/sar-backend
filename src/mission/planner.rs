use crate::mission::waypoint::Waypoint;
//use std::f64::consts::PI;

const EARTH_RADIUS_M: f64 = 6378.137e3;

// Haversine Distance between two (lat, lon) pair points in meters | Parameters: ((lat1, lon1), (lat2, lon2)) | (lat2, lon2) >= (lat1, lon1) respectively
pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64{
    let del_lat_rad: f64 = (lat2 - lat1).to_radians();
    let del_lon_rad: f64 = (lon2 - lon1).to_radians();
    let lat1_rad: f64 = lat1.to_radians();
    let lat2_rad: f64 = lat2.to_radians();

    let a = ((1.0 - del_lat_rad.cos()) / 2.0) + lat1_rad.cos() * lat2_rad.cos() * ((1.0 - del_lon_rad.cos()) / 2.0);
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_M * c
}


// | Lawnmower Traversal in a given rectangular slice 
pub fn lawnmower_slice_search (
    lat_min: f64,           //
    lon_min: f64,           //
    lat_max: f64,           //
    lon_max: f64,           //
    n_drones: u32,          //
    swath_width: f64,       //
    altitude: f64,          //
) -> Vec<Vec<Waypoint>> {
    let mut missions: Vec<Vec<Waypoint>> = Vec::new();

    let total_width = haversine_distance(lat_min, lon_min, lat_min, lon_max); //Total width in meters

    //println!("Total width in meters = {:.6}", total_width);

    let slice_width = total_width / n_drones as f64;  //Width per drone in meters
    // println!("Slice width in meters = {:.6}", slice_width);
    // println!("No. of waypoints per slice = {:.2}", slice_width/swath_width);
    

    let lon_per_m = (lon_max - lon_min) / total_width; //longitude per meter in width
    // println!("longitude per meter in width = {:.6}", lon_per_m);
    // println!("Swath Width in deg = {:.6}", swath_width * lon_per_m);

    // println!("No. of waypoints per slice = {:.2}", (0.01-0.00)/(swath_width * lon_per_m));

    for i in 0..n_drones {
        let mut waypoints: Vec<Waypoint> = Vec::new();

        let slice_min_lon = lon_min + i as f64 * slice_width * lon_per_m; 
        let slice_max_lon = lon_min + (i as f64 + 1.0) * slice_width * lon_per_m; 

        // let mut current_lon = slice_min_lon;
        // let mut traverse_up = true;

        // while current_lon <= slice_max_lon {
        //     let target_lat = if traverse_up {lat_max} else {lat_min};
        //     waypoints.push(Waypoint::new_waypoint(target_lat, current_lon, altitude));
        //     current_lon += swath_width * lon_per_m;
        //     traverse_up = !traverse_up
        // }

        let mut curr_long: f64 = slice_min_lon;
        let mut curr_lat: f64 = lat_min;
        let mut traverse_up = true;

        while curr_long < slice_max_lon {
            waypoints.push(Waypoint::new_waypoint(curr_lat, curr_long, altitude));
            if traverse_up {curr_lat = lat_max} else {curr_lat = lat_min};
            waypoints.push(Waypoint::new_waypoint(curr_lat, curr_long, altitude));
            curr_long += swath_width * lon_per_m;
            traverse_up = !traverse_up;

        }

        // let slice_lon_min = lon_min + i as f64 * slice_width * lon_per_m;
        // let slice_lon_max = lon_min + (i as f64 + 1.0) * slice_width * lon_per_m;

        // let mut current_lat = lat_min;
        // let mut going_up = true;

        // // Generate lawnmower pattern
        // while current_lat <= lat_max {
        //     let lon_target = if going_up { slice_lon_max } else { slice_lon_min };

        //     waypoints.push(Waypoint::new_waypoint(current_lat, lon_target, altitude));

        //     current_lat += (swath_width / EARTH_RADIUS_M) * (180.0 / PI);
        //     going_up = !going_up;
        // }
        
        missions.push(waypoints);
    }
    missions
    }




// --------------------------------------------------------------
// |  PARAMETERS       |  DATA TYPES             |  UNITS       |
// --------------------------------------------------------------
// |  drone_id         |  Integer     - u32      |  .......     | 
// |  latitude         |  Float       - f64      |  degrees     |
// |  longitude        |  Float       - f64      |  degrees     |
// |  altitude         |  Float       - f64      |  meters      |
// |  battery          |  Float       - f32      |  %           |
// |  connected        |  Boolean     - bool     |  T / F       |
// |  EARTH_RADIUS_M   |  Float       - f64      |  meters      |
// |  .........        |  .......     - ....     |  .......     |
// |  .........        |  .......     - ....     |  .......     |
// |  .........        |  .......     - ....     |  .......     |
// |  .........        |  .......     - ....     |  .......     |
// |  .........        |  .......     - ....     |  .......     |
// --------------------------------------------------------------


// Vertical trasveral for now in zig - zag manner 
// Want the search method to be a struct which can return the width, lon/lat per m, cell width etc.
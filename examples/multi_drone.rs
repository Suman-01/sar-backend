// use sar_backend::drone::{DroneManager};

// fn main() {
//     let mut mng = DroneManager::new_manager();

//     mng.add_drone(10);
//     mng.add_drone(11);
//     mng.add_drone(12);

//     if let Some(drone) = mng.get_state_mut(10) {
//         drone.establish_connection();
//         drone.update_position(1.0, 2.0, 3.0);
//         drone.upadate_battery(90.0);
//     }

//     if let Some(drone) = mng.get_state_mut(10) {
//         drone.establish_connection();
//         drone.update_position(11.0, 12.0, 13.0);
//         drone.upadate_battery(91.0);
//     }

//     if let Some(drone) = mng.get_state_mut(10) {
//         drone.establish_connection();
//         drone.update_position(111.0, 112.0, 113.0);
//         drone.upadate_battery(92.0);
//     }
    
//     for (drone_id, drone) in mng.drone_iterator() {
//         println!(
//             "Drone {} : Lat = {}, Lon = {}, Alt = {}, Bat = {}%, Conn = {}",
//             drone_id, drone.latitude, drone.longitude, drone.altitude, drone.battery, drone.connected
//         );

//     }



// }

fn main() {
    
}

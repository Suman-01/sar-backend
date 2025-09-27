use crate::drone::{DroneManager, DroneState};
use crate::comms::manager::CommsManager;
use crate::logging::{DataLoger, event::{EventType, Event}};
use crate::mission::{Waypoint, planner::lawnmower_slice_search};
use crate::coverage::{Grid, Cell};


pub struct SystemManager {
    pub drone: DroneManager,
    pub comms: CommsManager,
    pub logger: DataLoger,
    pub grid: Option<Grid>,
}

impl SystemManager {
    pub fn new_system_manager() -> Self {
        Self {
            drone: DroneManager::new_manager(),
            comms: CommsManager::new_comms_manager(),
            logger: DataLoger::new_dataloger(),
            grid: None,
        }
    }

    //Add a drone to comms and drone state separately 
    pub fn add_drone(&mut self, drone_id: u32) {
        self.drone.add_drone(drone_id);
        self.comms.add_drone(drone_id);
        self.logger.add_event(Event::new_event(drone_id, EventType::Other("Drone Added".to_string()), None, None));
    }



    //Add a mission / a set of Waypoints | for now only a logger
    pub fn add_mission(&mut self, drone_id: u32, waypoints: Vec<Waypoint>) {
        if let Some(drones) = self.drone.get_state_immut(drone_id) {
            self.logger.add_event(Event::new_event(drone_id, EventType::Other(format!("Mission assigned with waypoints: {}", waypoints.len())), Some(drones.get_latitude()), Some(drones.get_longitude())));
        }
    }

    pub fn update_drone_position(&mut self, drone_id: u32, latitude: f64, longitude: f64, altitude: f64) {
        if let Some(drones) = self.drone.get_state_mut(drone_id) {
            drones.update_position(latitude, longitude, altitude);
            self.logger.add_event(Event::new_event(drone_id, EventType::WaypointReached, Some(latitude), Some(longitude)));
        }
    }

    pub fn update_comms(&mut self, drone_id: u32, latency_ms: f32, packet_loss: f32, is_alive: bool) {
        self.comms.update_metrics(drone_id, latency_ms, packet_loss, is_alive);
    }

    pub fn get_logs(&self, drone_id: u32) -> Vec<&Event> {
        self.logger.get_drone_events(drone_id)
    }

    pub fn init_grid(&mut self, rows: u32, cols: u32) {
        self.grid = Some(Grid::new_grid(rows, cols))
    }

    pub fn list_drones(&self) -> Vec<u32> {
        self.drone.drone_iterator().map(|(&id, _)| id).collect()
    }

}

// Not a full integration | Just a manager to manage all managers | few functions to check inheritance and working | should work.
// No function to remove drone
// For now no encapsulations
// Also See that encapsulation makes many function in their respective files unusable until manager uses it or gives us another fn to use that fn
// So Keep fns in those directories which will be used by the manager only and rest fns declared by manager for public use.
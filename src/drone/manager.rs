use std::collections::HashMap;
use super::state::DroneState;

// Drone Manager Structure
pub struct DroneManager {
    drones: HashMap<u32, DroneState>,  // Private Drone Variables
}

impl DroneManager {

    // Create an Empty/New Drone manager Hashmap
    pub fn new_manager() -> Self{
        Self {
            drones: HashMap::new(),
        }
    }

    // Add new drones | Parameter: (drone_id)
    pub fn add_drone(&mut self, drone_id: u32) {
        self.drones.insert(drone_id, DroneState::new_drone(drone_id));
    }

    // Remove drones | Parameter: (drone_id)
    pub fn remove_drone(&mut self, drone_id: u32) {
        self.drones.remove(&drone_id);
    }

    // Immutable Reference to drone states | Parameter: (drone_id)
    pub fn get_state_immut(&self, drone_id: u32) -> Option<&DroneState> {
        self.drones.get(&drone_id)
    }

    // Mutable Reference to drone states | Parameter: (drone_id)
    // Might need to be removed (only for prototype)
    pub fn get_state_mut(&mut self, drone_id: u32) -> Option<&mut DroneState> {
        self.drones.get_mut(&drone_id)
    }

    // All drones iterator | Check if returns immutable references only
    pub fn drone_iterator(&self) -> impl Iterator<Item = (&u32, &DroneState)> {
        self.drones.iter()
    }

}




// ... | Parameters: ()

// -----------------------------------------------------------
// |  PARAMETERS    |  DATA TYPES             |  UNITS       |
// -----------------------------------------------------------
// |  drone_id      |  Integer     - u32      |  .......     | 
// |  drones        |  Hash Map    - ...      |  .......     |
// |  .........     |  .......     - ....     |  .......     |
// -----------------------------------------------------------


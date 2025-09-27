pub mod drone;
pub mod mission;
pub mod coverage;
pub mod comms;
pub mod logging;
pub mod system;



// Variables are not encapsulated | read-only getters | 
// waypoints only set to max and min lat in row so other rows will not be reached | change the planner for this

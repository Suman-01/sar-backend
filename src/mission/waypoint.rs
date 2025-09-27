#[derive(Debug, Clone)]

// | Waypoint (A single navigation target)
pub struct Waypoint {
    longitude: f64,     // Longitude 
    latitude: f64,      // Latitude 
    altitude: f64,      // Altitude 
    hover_time: f32,    // Hovering duration 
}

impl Waypoint {

    // Create a new Waypoint | Parameters: (lat, lon, alt)
    pub fn new_waypoint(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude, 
            altitude,
            hover_time: 0.0,
        }
    }

    // | Getters Start ---
    pub fn get_latitude(&self) -> f64 {self.latitude}
    pub fn get_longitude(&self) -> f64 {self.longitude}
    pub fn get_altitude(&self) -> f64 {self.altitude}
    // | Getters End   ---

    // Add/Change hovering duration | Parameter: (hover_time)
    pub fn update_hover_time(mut self, sec: f32) { self.hover_time = sec;}

}

// ... | Parameters: ()

// -----------------------------------------------------------
// |  PARAMETERS    |  DATA TYPES             |  UNITS       |
// -----------------------------------------------------------
// |  drone_id      |  Integer     - u32      |  .......     | 
// |  latitude      |  Float       - f64      |  degrees     |
// |  longitude     |  Float       - f64      |  degrees     |
// |  altitude      |  Float       - f64      |  meters      |
// |  hover_time    |  Float       - f32      |  seconds     | <--*
// |  .........     |  .......     - ....     |  .......     |
// -----------------------------------------------------------


// | How do you check the waypoints | Do you call some get_waypoint func? | Do you call it after every update? 
// | Is there a way to show the changes or link it to the get_waypoint func to call it automatically?
// | No way to remove waypoint






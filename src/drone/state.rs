#[derive(Debug, Clone)]

pub struct DroneState {
    drone_id: u32,      // Drone ID 
    latitude: f64,      // Latitude
    longitude: f64,     // Longitude 
    altitude: f64,      // Altitude  
    battery: f32,       // Battery level 
    is_connected: bool,    // Connetion State 
}

impl DroneState {

    // Make a new DroneState with default values | Parameters: (drone_id)
    pub fn new_drone(drone_id: u32) -> Self {
        Self {
            drone_id,
            latitude: 0.0,
            longitude: 0.0,
            altitude: 0.0,
            battery: 100.0,
            is_connected: false,
        }
    }
    
    // | Getter Files for Struct Parameters --------
    pub fn get_drone_id(&self) -> u32 {
        self.drone_id
    }

    pub fn get_latitude(&self) -> f64 {
        self.latitude
    }

    pub fn get_longitude(&self) -> f64 {
        self.longitude
    }

    pub fn get_altitude(&self) -> f64 {
        self.altitude
    }

    pub fn get_battery(&self) -> f32 {
        self.battery
    }

    pub fn is_connected(&self) -> bool {
        self.is_connected
    }
    // | Getter files end --------------------------------
    

    // | Controlled Updates | The clampings might need to be changed (only prototype)

    // Establish connecion for this particular DroneState
    pub fn connect_drone(&mut self) {
        self.is_connected = true;
    }

    // Terminate connecion for this particular DroneState
    pub fn disconnect_drone(&mut self) {
        self.is_connected = false;
    }

    // Update Drone position | Parameters: (lat, lon, alt)
    pub fn update_position(&mut self, latitude: f64, longitude: f64, altitude: f64) {
        self.latitude = latitude.clamp(-90.0, 90.0);
        self.longitude = longitude.clamp(-180.0, 180.0);
        self.altitude = altitude.max(0.0);
    }

    // Update battery percentage | Parameters: (battery)
    pub fn upadate_battery(&mut self, battery: f32) {
        self.battery = battery.clamp(0.0, 100.0);   // clamping is wrong thing to do here
        
    }

}

// Do we really need drone_id in drone states



// ... | Parameters: ()

// -----------------------------------------------------------
// |  PARAMETERS    |  DATA TYPES             |  UNITS       |
// -----------------------------------------------------------
// |  drone_id      |  Integer     - u32      |  .......     | 
// |  latitude      |  Float       - f64      |  degrees     |
// |  longitude     |  Float       - f64      |  degrees     |
// |  altitude      |  Float       - f64      |  meters      |
// |  battery       |  Float       - f32      |  %           |
// |  is_connected  |  Boolean     - bool     |  T / F       |
// |  .........     |  .......     - ....     |  .......     |
// -----------------------------------------------------------

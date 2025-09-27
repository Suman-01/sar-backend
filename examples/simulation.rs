use sar_backend::{
    coverage::{Grid}, drone::DroneManager, mission::{haversine_distance, lawnmower_slice_search, Waypoint}
    // comms::CommsManager,
    // logging::DataLoger,
};


pub struct Simulation {
    grid: Grid,
    drone_manager: DroneManager,
    mission_waypoints: Vec<Vec<Waypoint>>,
    lat_min: f64,
    lat_max: f64,
    lon_min: f64,
    lon_max: f64,

}

impl Simulation {
    pub fn new_simulation(
        lat_min: f64,
        lat_max: f64,
        lon_min: f64,
        lon_max: f64,
        n_drones: u32,
        swath_width: f64,   // in m
        altitude: f64,  // in m
    ) -> Self {
        let mission_waypoints = lawnmower_slice_search(lat_min, lon_min, lat_max, lon_max, n_drones, swath_width, altitude);
        let total_width = haversine_distance(lat_min, lon_min, lat_min, lon_max); //in m
        let slice_width = total_width / n_drones as f64;
        //println!("Total width = {:.6}", total_width);
        let cols = (n_drones * ((slice_width / swath_width) + 1.0) as u32) * 2;
        let rows: u32 = 3;
        //println!("Total cols = {:.6}", cols);

        // Grid with 2 rows and cols (total_width / swath_width)
        let grid = Grid::new_grid(rows, cols);


        // Init the drones
        let mut drone_manager = DroneManager::new_manager();
        for drone_id in 0..=n_drones {
            drone_manager.add_drone(drone_id);
        }

        Self {grid, drone_manager, mission_waypoints, lat_min, lat_max, lon_min, lon_max}

    }

    pub fn start_simulation(&mut self) {
        println!("Starting Simulation...");
        let max_steps = self.mission_waypoints.iter().map(|m| m.len()).max().unwrap_or(0);

        for step in 0..max_steps {
            for (drone_id, waypoints) in self.mission_waypoints.iter().enumerate() {
                if step < waypoints.len() {
                    let wp = &waypoints[step];

                    // Update position
                    if let Some(drone) = self.drone_manager.get_state_mut(drone_id as u32) {
                        drone.update_position(wp.get_latitude(), wp.get_longitude(), wp.get_altitude());
                    }

                    let cell_height = (self.lat_max - self.lat_min) / (self.grid.rows() - 1) as f64;
                    let cell_width = (self.lon_max - self.lon_min) / self.grid.cols() as f64;

                    //Convert waypoints to Gridcells
                    let row = ((wp.get_latitude() - self.lat_min) / cell_height).floor() as u32; 
                    let col = ((wp.get_longitude() - self.lon_min) / cell_width).floor() as u32;
                    //println!("drone_id = {}  |  (row, col) = {}, {})", drone_id, row, col);

                    // println!("drone_id = {}  |  (longitude, latitude) = ({:.6}, {:.6})", drone_id, wp.get_longitude(), wp.get_latitude());

                    self.grid.visit_gridcell(row, col);
                }
            }
            self.print_grid()
        }
    }

    fn print_grid(&self) {
        //println!("Step {}: ", step + 1);
        for r in 0..self.grid.rows() {
            for c in 0..self.grid.cols() {
                let symbol = if let Some(cell) = self.grid.get_cell(r, c) {
                    if cell.visited {"✔"} else {"·"}
                } else {
                    "·"
                };
                print!("{} ", symbol);
            }
            println!();
        }
        println!();
    }
}

fn main() {
    let lat_min = 0.0;
    let lon_min = 0.0;
    let lat_max = 0.02;
    let lon_max = 0.02;

    let n_drones = 2;
    let swath_width = 100.0;   // distance between scan lines
    let altitude = 50.0;   // meters

    let mut sim = Simulation::new_simulation(lat_min, lat_max, lon_min, lon_max, n_drones, swath_width, altitude);
    sim.start_simulation();
}
  



use geometry_msgs::msg::Pose;
use nav_msgs::msg::{MapMetaData, OccupancyGrid, Odometry};
use px4_msgs::msg::VehicleGlobalPosition;
use rclrs::*;
use std::error::Error;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

const PUBLISH_HZ: u64 = 10;
const RESOLUTION_M: f64 = 0.5;
const PAINT_RADIUS_M: f64 = 0.8;

const GLOBAL_CORNERS: [(f64, f64); 4] = [
    (37.412308, -121.998881),
    (37.412097, -121.998693),
    (37.412462, -121.998331),
    (37.412270, -121.998189),
];

fn mpd(lat_deg: f64) -> (f64, f64) {
    let lat_rad = lat_deg.to_radians();
    let lat_m = 111132.954 - 559.822 * (2.0 * lat_rad).cos()
        + 1.175 * (4.0 * lat_rad).cos()
        - 0.0023 * (6.0 * lat_rad).cos();
    let lon_m = 111412.84 * lat_rad.cos()
        - 93.5 * (3.0 * lat_rad).cos()
        + 0.118 * (5.0 * lat_rad).cos();
    (lat_m, lon_m)
}

fn geo_to_enu(lat: f64, lon: f64, lat_o: f64, lon_o: f64) -> (f64, f64) {
    let (lat_mpd, lon_mpd) = mpd(lat_o);
    let n = (lat - lat_o) * lat_mpd;
    let e = (lon - lon_o) * lon_mpd;
    (e, n)
}

fn ordered_polygon_local() -> Vec<(f64, f64)> {
    let lat_o = GLOBAL_CORNERS[0].0;
    let lon_o = GLOBAL_CORNERS[0].1;

    let perimeter = [
        GLOBAL_CORNERS[0],
        GLOBAL_CORNERS[2],
        GLOBAL_CORNERS[3],
        GLOBAL_CORNERS[1],
    ];

    perimeter
        .iter()
        .map(|(lat, lon)| geo_to_enu(*lat, *lon, lat_o, lon_o))
        .collect()
}

fn point_in_poly(p: (f64, f64), poly: &[(f64, f64)]) -> bool {
    let (x, y) = p;
    let mut inside = false;
    let n = poly.len();

    for i in 0..n {
        let (x1, y1) = poly[i];
        let (x2, y2) = poly[(i + 1) % n];

        let intersects = ((y1 > y) != (y2 > y))
            && (x < (x2 - x1) * (y - y1) / ((y2 - y1).abs().max(1e-12)) + x1);

        if intersects {
            inside = !inside;
        }
    }

    inside
}

#[derive(Clone, Copy, Default)]
struct DronePose {
    e: f64,
    n: f64,
    u: f64,
    valid: bool,
}

#[derive(Clone)]
struct CoverageMap {
    grid: OccupancyGrid,
    origin_e: f64,
    origin_n: f64,
    width: u32,
    height: u32,
    resolution: f64,
}

impl CoverageMap {
    fn new() -> Self {
        let poly = ordered_polygon_local();

        let min_e = poly.iter().map(|p| p.0).fold(f64::INFINITY, f64::min);
        let max_e = poly.iter().map(|p| p.0).fold(f64::NEG_INFINITY, f64::max);
        let min_n = poly.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let max_n = poly.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);

        let width = ((max_e - min_e) / RESOLUTION_M).ceil() as u32 + 4;
        let height = ((max_n - min_n) / RESOLUTION_M).ceil() as u32 + 4;

        let origin_e = min_e - RESOLUTION_M * 2.0;
        let origin_n = min_n - RESOLUTION_M * 2.0;

        let mut grid = OccupancyGrid::default();
        grid.header.frame_id = "map".to_string();
        grid.info = MapMetaData::default();
        grid.info.resolution = RESOLUTION_M as f32;
        grid.info.width = width;
        grid.info.height = height;

        let mut origin = Pose::default();
        origin.position.x = origin_e;
        origin.position.y = origin_n;
        origin.position.z = 0.0;
        origin.orientation.w = 1.0;
        grid.info.origin = origin;

        let mut data = vec![-1_i8; (width * height) as usize];

        for row in 0..height {
            for col in 0..width {
                let e = origin_e + (col as f64 + 0.5) * RESOLUTION_M;
                let n = origin_n + (row as f64 + 0.5) * RESOLUTION_M;
                let idx = (row * width + col) as usize;

                if point_in_poly((e, n), &poly) {
                    data[idx] = 0;
                }
            }
        }

        grid.data = data;

        Self {
            grid,
            origin_e,
            origin_n,
            width,
            height,
            resolution: RESOLUTION_M,
        }
    }

    fn world_to_cell(&self, e: f64, n: f64) -> Option<(i32, i32)> {
        let col = ((e - self.origin_e) / self.resolution).floor() as i32;
        let row = ((n - self.origin_n) / self.resolution).floor() as i32;

        if col < 0 || row < 0 {
            return None;
        }
        if col >= self.width as i32 || row >= self.height as i32 {
            return None;
        }
        Some((row, col))
    }

    fn paint_disk(&mut self, e: f64, n: f64, radius_m: f64, value: i8) {
        let Some((row0, col0)) = self.world_to_cell(e, n) else {
            return;
        };

        let radius_cells = (radius_m / self.resolution).ceil() as i32;

        for dr in -radius_cells..=radius_cells {
            for dc in -radius_cells..=radius_cells {
                if dr * dr + dc * dc > radius_cells * radius_cells {
                    continue;
                }

                let row = row0 + dr;
                let col = col0 + dc;

                if row < 0 || col < 0 {
                    continue;
                }
                if row >= self.height as i32 || col >= self.width as i32 {
                    continue;
                }

                let idx = (row as u32 * self.width + col as u32) as usize;
                self.grid.data[idx] = value;
            }
        }
    }

    fn mark_visit(&mut self, e: f64, n: f64) {
        self.paint_disk(e, n, PAINT_RADIUS_M, 100);
    }
}

struct Shared {
    map: CoverageMap,
    d1: DronePose,
    d2: DronePose,
}

fn make_odom(frame: &str, child: &str, e: f64, n: f64, u: f64) -> Odometry {
    let mut o = Odometry::default();
    o.header.frame_id = frame.to_string();
    o.child_frame_id = child.to_string();
    o.pose.pose.position.x = e;
    o.pose.pose.position.y = n;
    o.pose.pose.position.z = u;
    o.pose.pose.orientation.w = 1.0;
    o
}

pub fn run() -> Result<(), Box<dyn Error>> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("coverage_viz")?;

    let state = Arc::new(Mutex::new(Shared {
        map: CoverageMap::new(),
        d1: DronePose::default(),
        d2: DronePose::default(),
    }));

    let grid_pub = node.create_publisher::<OccupancyGrid>("/mission_grid")?;
    let odom1_pub = node.create_publisher::<Odometry>("/px4_1/odom_map")?;
    let odom2_pub = node.create_publisher::<Odometry>("/px4_2/odom_map")?;

    {
        let state = state.clone();
        node.create_subscription(
            "/px4_1/fmu/out/vehicle_global_position",
            move |msg: VehicleGlobalPosition| {
                let lat = msg.lat as f64;
                let lon = msg.lon as f64;
                let lat_o = GLOBAL_CORNERS[0].0;
                let lon_o = GLOBAL_CORNERS[0].1;
                let (e, n) = geo_to_enu(lat, lon, lat_o, lon_o);

                let mut s = state.lock().unwrap();
                s.map.mark_visit(e, n);
                s.d1 = DronePose {
                    e,
                    n,
                    u: 0.0,
                    valid: true,
                };
            },
        )?;
    }

    {
        let state = state.clone();
        node.create_subscription(
            "/px4_2/fmu/out/vehicle_global_position",
            move |msg: VehicleGlobalPosition| {
                let lat = msg.lat as f64;
                let lon = msg.lon as f64;
                let lat_o = GLOBAL_CORNERS[0].0;
                let lon_o = GLOBAL_CORNERS[0].1;
                let (e, n) = geo_to_enu(lat, lon, lat_o, lon_o);

                let mut s = state.lock().unwrap();
                s.map.mark_visit(e, n);
                s.d2 = DronePose {
                    e,
                    n,
                    u: 0.0,
                    valid: true,
                };
            },
        )?;
    }

    {
        let state = state.clone();
        thread::spawn(move || loop {
            let (grid, d1, d2) = {
                let s = state.lock().unwrap();
                (s.map.grid.clone(), s.d1, s.d2)
            };

            grid_pub.publish(&grid).ok();

            if d1.valid {
                let o = make_odom("map", "px4_1/base_link", d1.e, d1.n, d1.u);
                odom1_pub.publish(&o).ok();
            }

            if d2.valid {
                let o = make_odom("map", "px4_2/base_link", d2.e, d2.n, d2.u);
                odom2_pub.publish(&o).ok();
            }

            thread::sleep(Duration::from_millis(1000 / PUBLISH_HZ));
        });
    }

    thread::spawn(move || {
        let _ = executor.spin(SpinOptions::default());
    });

    Ok(())
}
mod collision_avoidance;

use rlrs::*;
use px4_msgs::msg::{TrajectorySetpoint, VehicleCommand, OffboardControl, VehicleGlobalPosition};
use std::sync::{Arc, Mutex};
use std::time::{Duration, UNIX_EPOCH, SystemTime};
use std::thread::{self, yield_now};
use std::error::Error;
use collision_avoidance::CollisionAvoidance;


const GLOBAL_CORNERS:[(f64, f64); 4] = [
    (37.412308, -121.998881), //top left 
    (37.412097, -121.998693), //bottom left 
    (37.412462, -121.998331), //top right 
    (37.412270, -121.998189), //bottom right
];

const ALTITUDE: f64 = 5.0; //in meters
const STEP: f64 = 2.0; //in meters
const PUBLISH_PERIOD_MS: u64 = 200; //setpoint publish period in ms
const REACHED_THRES_M: f64 = 1.0; //has reached thres

//mpd: meters per degree | convert deg to meters
fn mpd(lat: f64) -> (f64, f64) {
    let lat_mpd = 111_133.0_f64;
    let lon_mpd = lat_mpd * lat.to_radians().cos();
    (lat_mpd, lon_mpd)
}

fn timestamp_us() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_micros() as u64
}

//reference to origin (lat0, lon0)
//test it with local_to_geo
fn geo_to_local(lat: f64, lon: f64, lat_o: f64, lon_o: f64) -> (f64, f64) {
    let (lat_mpd, lon_mpd) = mpd(lat0);
    let dn = (lat - lat_o) * lat_mpd; //north
    let de = (lon - lon_o) * lon_mpd; //east
    (dn, de) 
}

fn distance_mean(
    lat_a: f64,
    lon_a: f64,
    lat_b: f64,
    lon_b: f64,
    lat_o: f64, //origin
    lon_o: f64, //origin
) -> f64 {
    let (ax, ay) = geo_to_local(lat_a, lon_a, lat_o, lon_o);
    let (bx, by) = geo_to_local(lat_b, lon_b, lat_o, lon_o);
    let dx = ax - bx;
    let dy = ay - by;
    (dx*dx + dy*dy).sqrt()
}

fn midpt (a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    ((a.0 + b.0)/2.0, (a.1 + b.1)/2.0)
}

fn setpoint(n: f64, e:f64, d:f64, yaw:f64) -> TrajectorySetpoint {
    TrajectorySetpoint {
        timestamp: timestamp_us(),
        position: [n as f32, e as f32, d as f32],
        velocity: [f32::NAN, f32::NAN, f32::NAN],
        acceleration: [f32::NAN, f32::NAN, f32::NAN],
        jerk: [0.0, 0.0, 0.0],
        yaw: yaw as f32,
        yawspeed: 0.0,
    }
}

//agian and again same input vars???
fn pub_setpoint(
    publisher: &Publisher<TrajectorySetpoint>,
    lat: f64,
    lon: f64,
    alt_m: f64,
    lat_o: f64,
    lon_o: f64,
) {
    let (n, e) = geo_to_local(lat, lon, lat_o, lon_o);
    let msg = setpoint(n, e, -alt_m, 0.0);
    let _ = publisher.publish(msg);
}

fn straight_path(
    start: (f64, f64),
    target: (f64, f64),
    lat_o: f64,
    lon_o: f64,
    step_m: f64,   // <-*
    alt_m: f64,
    publisher: &Publisher<TrajectorySetpoint>,
    avoid: &CollisionAvoidance,
) {
    let (sx, sy) = geo_to_local(start.0, start.1, lat_o, lon_o);
    let (tx, ty) = geo_to_local(target.0, target.1, lat_o, lon_o);
    let dx = tx - sx;
    let dy = ty - dy;
    let dist = (dx*dx + dy*dy).sqrt();

    if dist < 1e-6 {
        let _ = publisher.publish(setpoint(sx, sy, -alt_m, 0.0));
        return;
    }

    let ux = dx / dist;
    let uy = dy / dist;
    let mut s = step_m;
// might not be a factor of step size and go ahead of the corners
    while s < dist {
        let nx = sx + ux*s;
        let ny = sy + uy*s;
        let yaw = uy*atran2(ux); //<-* ??

        let (ax, ay) = avoid.avoidance_vec();
        
        nx += ax;
        ny += ay;

        let _ = publisher.publish(&setpoint(nx, ny, -alt_m, yaw));

        thread::sleep(Duration::from_millis(PUBLISH_PERIOD_MS));
        s += step_m;
    }

    let yaw = dy.atan2(dx);
    let _ = publisher.publish(setpoint(tx, ty, -alt_m, yaw));

}

pub fn run_mission(node: &Node) -> Result<(), Box<dyn Error>> {
    let drone1_pos: Arc<Mutex<Option<(f64, f64)>>> = Arc::new(Mutex::new(None));
    let drone2_pos: Arc<Mutex<Option<(f64, f64)>>> = Arc::new(Mutex::new(None));

    let pub_drone1 = node.create_publisher::<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint")?;
    let pub_drone2 = node.create_publisher::<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint")?;

    let avoid = CollisionAvoidance::new();

    let _lidar_sub = {
        let a = avoid.scan.clone();

        node.create_subscription::<LaserScan, _>(
            "/scan",
            move |msg| {
                *a.lock().unwrap() = Some(msg);
            },
        )?
    };

    {
        let d1 = drone1_pos.clone();
        node.create_subscription::<VehicleGlobalPosition, _>(
            "/px4_1/fmu/out/vehicle_global_position",
            move |msg| {
                *d1.lock().unwrap() = Some((msg.lat, msg.lon));
            },
        )?;
    }

    {
        let d2 = drone2_pos.clone();
        node.create_subscription::<VehicleGlobalPosition, _>(
            "/px4_2/fmu/out/vehicle_global_position",
            move |msg| {
                *d2.lock().unwrap() = Some((msg.lat, msg.lon));
            },
        )?;
    }

    let top_left = GLOBAL_CORNERS[0];
    let bottom_left = GLOBAL_CORNERS[1];
    let top_right = GLOBAL_CORNERS[2];
    let bottom_right = GLOBAL_CORNERS[3];

    let lat_o = top_left.0;
    let lon_o = top_left.1;

    let mid_left = midpt(top_left, bottom_left);
    let mid_right = midpt(top_right, bottom_right);

    let mid_top_right = midpt(top_right, mid_right);
    let mid_bottom_right = midpt(mid_right, bottom_right);

    let mid_top_left = midpt(top_left, mid_left);
    let mid_bottom_left = midpt(mid_left, bottom_left);

    let has_reached = |arc: &Arc<Mutex<Option<(f64, f64)>>>, target: (f64, f64)| -> bool {
        if let Some((lat, lon)) = *arc.lock().unwrap() {
            distance_mean(lat, lon, target.0, target.1, lat_o, lon_o) <= REACHED_THRES_M
        } else {
            false
        }
    };

    let wait = |arc: &Arc<Mutex<Option<(f64, f64)>>>, target: (f64, f64)| loop {
        if has_reached(arc, target) {
            break;
        }
        thread::sleep(Duration::from_millis(200));
    };

    println!("START MISSION!!!");

    pub_setpoint(&pub_drone1, top_left.0, top_left.1, ALTITUDE, lat_o, lon_o);
    wait(&drone1_pos, top_left); //<-* drone1_pos does it have a value or d1 has the value

    pub_setpoint(&pub_drone2, mid_left.0, mid_left.1, ALTITUDE, lat_o, lon_o);
    wait(&drone2_pos, mid_left); //<-* drone2_pos does it have a value or d2 has the value

    straight_path(top_left, top_right, lat_o, lon_o, STEP, ALTITUDE, &pub_drone1, &avoid);
    straight_path(mid_left, mid_right, lat_o, lon_o, STEP, ALTITUDE, &pub_drone2, &avoid);

    straight_path(top_right, mid_top_right, lat_o, lon_o, STEP, ALTITUDE, &pub_drone1, &avoid);
    straight_path(mid_right, mid_bottom_right, lat_o, lon_o, STEP, ALTITUDE, &pub_drone2, &avoid);

    straight_path(mid_top_right, mid_top_left, lat_o, lon_o, STEP, ALTITUDE, &pub_drone1, &avoid);
    straight_path(mid_bottom_right, mid_bottom_left, lat_o, lon_o, STEP, ALTITUDE, &pub_drone2, &avoid);

    println!("MISSION COMPLETE!!!");
    Ok(())

}
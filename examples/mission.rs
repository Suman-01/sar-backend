// mission.rs
use crate::offboard::OffboardController;
use crate::arm::ArmDrone;
use crate::take_off::TakeOff;
use rclrs::Node;
use std::thread;
use std::error::Error;
use std::sync::Arc;
use std::time::Duration;

const ALTITUDE: f64 = 5.0; // meters
const STEP: f64 = 2.0; // meters per step
const SPEED_MPS: f64 = 0.8; // meters per second (tune to slow / speed up)
const PUBLISH_PERIOD_MS: u64 = 100;

// Four global corners (lat, lon)
const GLOBAL_CORNERS: [(f64, f64); 4] = [
    (37.412308, -121.998881), // top left
    (37.412097, -121.998693), // bottom left
    (37.412462, -121.998331), // top right
    (37.412270, -121.998189), // bottom right
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

/// convert geographic (lat, lon) to local meters using origin (lat_o, lon_o)
fn geo_to_local(lat: f64, lon: f64, lat_o: f64, lon_o: f64) -> (f64, f64) {
    let (lat_mpd, lon_mpd) = mpd(lat_o);
    let dn = (lat - lat_o) * lat_mpd; // north (meters)
    let de = (lon - lon_o) * lon_mpd; // east  (meters)
    (dn, de)
}

fn midpt(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0)
}

/// publish a single-goal and sleep estimated time for arrival (distance / SPEED_MPS + margin)
fn goto_latlon_estimate(
    controller: &OffboardController,
    lat: f64,
    lon: f64,
    lat_o: f64,
    lon_o: f64,
    alt_m: f64,
) {
    let (x, y) = geo_to_local(lat, lon, lat_o, lon_o);
    let dist = (x * x + y * y).sqrt();

    let yaw = 0.0; // we don't care for this intermediate goto
    controller.set_target(x, y, -alt_m, yaw);

    // estimate travel time
    let t_s = (dist / SPEED_MPS).max((PUBLISH_PERIOD_MS as f64) / 1000.0) + 1.0; // +1s margin
    println!("goto estimate: dist {:.2} m -> sleeping {:.2} s", dist, t_s);
    thread::sleep(Duration::from_secs_f64(t_s));
}

/// straight, stepped path between two lat/lon points (start and target are lat/lon)
fn straight_path(
    controller: &OffboardController,
    start: (f64, f64),
    target: (f64, f64),
    lat_o: f64,
    lon_o: f64,
    step_m: f64,
    alt_m: f64,
) {
    let (sx, sy) = geo_to_local(start.0, start.1, lat_o, lon_o);
    let (tx, ty) = geo_to_local(target.0, target.1, lat_o, lon_o);
    let dx = tx - sx;
    let dy = ty - sy;
    let dist = (dx * dx + dy * dy).sqrt();

    if dist < 1e-6 {
        controller.set_target(sx, sy, -alt_m, 0.0);
        thread::sleep(Duration::from_millis(PUBLISH_PERIOD_MS));
        return;
    }

    let ux = dx / dist;
    let uy = dy / dist;
    let yaw = dy.atan2(dx);

    // compute a sensible per-step sleep so ground speed ≈ SPEED_MPS
    let step_time_s = (step_m / SPEED_MPS).max((PUBLISH_PERIOD_MS as f64) / 1000.0);

    println!(
        "straight_path: {:.2} m total, step {:.2} m, step_time {:.2} s",
        dist, step_m, step_time_s
    );

    let mut s = step_m;
    while s < dist {
        let nx = sx + ux * s;
        let ny = sy + uy * s;
        controller.set_target(nx, ny, -alt_m, yaw);
        thread::sleep(Duration::from_secs_f64(step_time_s));
        s += step_m;
    }

    // final point
    controller.set_target(tx, ty, -alt_m, yaw);
    // small hold
    thread::sleep(Duration::from_secs_f64(1.5));
}

/// Waits an estimated time for a path between these two lat/lon (used as a small helper)
fn _wait_path_estimate(a: (f64, f64), b: (f64, f64), lat_o: f64, lon_o: f64) {
    let (ax, ay) = geo_to_local(a.0, a.1, lat_o, lon_o);
    let (bx, by) = geo_to_local(b.0, b.1, lat_o, lon_o);
    let dx = bx - ax;
    let dy = by - ay;
    let dist = (dx * dx + dy * dy).sqrt();
    let t = (dist / SPEED_MPS).max(0.5) + 1.0;
    println!("wait_path_estimate: dist {:.2} m -> sleeping {:.2} s", dist, t);
    thread::sleep(Duration::from_secs_f64(t));
}

/// public mission entrypoint. Uses Arcs so orchestrator can pass shared controllers/arms
pub fn run_mission(
    _node: &Node,
    ctrl1: Arc<OffboardController>, // px4_1
    ctrl2: Arc<OffboardController>, // px4_2
    arm1: Arc<ArmDrone>,            // px4_1 arm/commands
    arm2: Arc<ArmDrone>,            // px4_2 arm/commands
) -> Result<(), Box<dyn Error>> {
    println!("MISSION START (coordinated dual-drone)");

    // corners
    let top_left = GLOBAL_CORNERS[0];
    let bottom_left = GLOBAL_CORNERS[1];
    let top_right = GLOBAL_CORNERS[2];
    let bottom_right = GLOBAL_CORNERS[3];

    // choose origin lat_o/lon_o (top_left)
    let lat_o = top_left.0;
    let lon_o = top_left.1;

    // midpoints per your plan
    let mid_left = midpt(top_left, bottom_left);
    let mid_right = midpt(top_right, bottom_right);

    let mid_top_right = midpt(top_right, mid_right);
    let mid_bottom_right = midpt(mid_right, bottom_right);

    let mid_top_left = midpt(top_left, mid_left);
    let mid_bottom_left = midpt(mid_left, bottom_left);

    // ---------------------------
    // Step 1: px4_1 arm + offboard + takeoff, go to top_left
    // ---------------------------
    arm1.set_offboard();
    arm1.arm();
    // takeoff px4_1
    TakeOff::takeoff(&ctrl1);

    // instruct px4_1 to go to top_left (from its current takeoff origin)
    goto_latlon_estimate(&ctrl1, top_left.0, top_left.1, lat_o, lon_o, ALTITUDE);

    // Wait a short moment to ensure stable
    thread::sleep(Duration::from_secs(1));

    // ---------------------------
    // Step 2: once px4_1 is at top_left (estimated), bring px4_2 up and goto mid_left
    // ---------------------------
    // Now enable px4_2
    arm2.set_offboard();
    arm2.arm();
    // takeoff px4_2
    TakeOff::takeoff(&ctrl2);

    // Have px4_2 go to mid_left (start at takeoff origin)
    goto_latlon_estimate(&ctrl2, mid_left.0, mid_left.1, lat_o, lon_o, ALTITUDE);

    // ---------------------------
    // Step 3: parallel traversals
    // ---------------------------
    // px4_1 sequence
    let c1 = ctrl1.clone();
    let seq1 = thread::spawn(move || {
        // px4_1 plan
        straight_path(&c1, top_left, top_right, lat_o, lon_o, STEP, ALTITUDE);
        straight_path(&c1, top_right, mid_top_right, lat_o, lon_o, STEP, ALTITUDE);
        straight_path(&c1, mid_top_right, mid_top_left, lat_o, lon_o, STEP, ALTITUDE);
        println!("px4_1 traversal finished");
    });

    // px4_2 sequence
    let c2 = ctrl2.clone();
    let seq2 = thread::spawn(move || {
        straight_path(&c2, mid_left, mid_right, lat_o, lon_o, STEP, ALTITUDE);
        straight_path(&c2, mid_right, mid_bottom_right, lat_o, lon_o, STEP, ALTITUDE);
        straight_path(&c2, mid_bottom_right, mid_bottom_left, lat_o, lon_o, STEP, ALTITUDE);
        println!("px4_2 traversal finished");
    });

    // wait both finish
    seq1.join().ok();
    seq2.join().ok();

    // ---------------------------
    // Step 4: land and disarm both vehicles
    // ---------------------------
    println!("Both traversals complete — landing both UAVs");

    // Request land then disarm after a short wait
    arm1.land();
    arm2.land();

    // give time to land (conservative)
    thread::sleep(Duration::from_secs(8));

    arm1.disarm();
    arm2.disarm();

    println!("Mission COMPLETE (both landed and disarmed)");
    Ok(())
}

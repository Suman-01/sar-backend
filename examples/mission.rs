//mission.rs

use crate::arm::ArmDrone;
use crate::offboard::OffboardController;
use rclrs::Node;
use std::cmp::Ordering;
use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

const BASE_ALT: f64 = 5.0;
const DRONE2_ALT: f64 = 8.0; // vertical separation
const STEP_M: f64 = 2.0;
const SPEED_MPS: f64 = 1.5;
const PUBLISH_PERIOD_MS: u64 = 100;

// Geofence corners in geographic coordinates.
const GLOBAL_CORNERS: [(f64, f64); 4] = [
    (37.412308, -121.998881), // top left
    (37.412097, -121.998693), // bottom left
    (37.412462, -121.998331), // top right
    (37.412270, -121.998189), // bottom right
];

// seeds (hyper-parameter) 12 regions for now
const SEED_ROWS: usize = 3;
const SEED_COLS: usize = 4; 

// Dense sampling inside the geofence.
const SAMPLE_ROWS: usize = 18;
const SAMPLE_COLS: usize = 18;

#[derive(Clone, Copy, Debug)]
struct Pose {
    lat: f64,
    lon: f64,
    u: f64,
    v: f64,
    x: f64, // north (m)
    y: f64, // east  (m)
}

#[derive(Clone, Debug)]
struct Cell {
    pose: Pose,
    region_id: usize,
}

#[derive(Clone, Debug)]
struct RegionPlan {
    id: usize,
    row: usize,
    col: usize,
    cells: Vec<Cell>,
    center: Pose,
}

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

fn geo_to_local(lat: f64, lon: f64, lat_o: f64, lon_o: f64) -> (f64, f64) {
    let (lat_mpd, lon_mpd) = mpd(lat_o);
    let dn = (lat - lat_o) * lat_mpd;
    let de = (lon - lon_o) * lon_mpd;
    (dn, de)
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn bilerp(
    top_left: (f64, f64),
    bottom_left: (f64, f64),
    top_right: (f64, f64),
    bottom_right: (f64, f64),
    u: f64,
    v: f64,
) -> (f64, f64) {
    let top = (
        lerp(top_left.0, top_right.0, u),
        lerp(top_left.1, top_right.1, u),
    );
    let bottom = (
        lerp(bottom_left.0, bottom_right.0, u),
        lerp(bottom_left.1, bottom_right.1, u),
    );
    (lerp(top.0, bottom.0, v), lerp(top.1, bottom.1, v))
}

fn pose_from_uv(
    u: f64,
    v: f64,
    top_left: (f64, f64),
    bottom_left: (f64, f64),
    top_right: (f64, f64),
    bottom_right: (f64, f64),
    lat_o: f64,
    lon_o: f64,
) -> Pose {
    let (lat, lon) = bilerp(top_left, bottom_left, top_right, bottom_right, u, v);
    let (x, y) = geo_to_local(lat, lon, lat_o, lon_o);
    Pose { lat, lon, u, v, x, y }
}

fn region_seed_uvs() -> Vec<(usize, usize, f64, f64)> {
    let mut seeds = Vec::with_capacity(SEED_ROWS * SEED_COLS);
    for r in 0..SEED_ROWS {
        for c in 0..SEED_COLS {
            let u = (c as f64 + 0.5) / SEED_COLS as f64;
            let v = (r as f64 + 0.5) / SEED_ROWS as f64;
            seeds.push((r, c, u, v));
        }
    }
    seeds
}

fn build_cells(
    top_left: (f64, f64),
    bottom_left: (f64, f64),
    top_right: (f64, f64),
    bottom_right: (f64, f64),
    lat_o: f64,
    lon_o: f64,
) -> Vec<Cell> {
    let seeds = region_seed_uvs();
    let mut cells = Vec::with_capacity(SAMPLE_ROWS * SAMPLE_COLS);

    for r in 0..SAMPLE_ROWS {
        let v = (r as f64 + 0.5) / SAMPLE_ROWS as f64;
        for c in 0..SAMPLE_COLS {
            let u = (c as f64 + 0.5) / SAMPLE_COLS as f64;
            let pose = pose_from_uv(u, v, top_left, bottom_left, top_right, bottom_right, lat_o, lon_o);

            let mut best_id = 0usize;
            let mut best_d2 = f64::INFINITY;

            for (sid, (_, _, su, sv)) in seeds.iter().enumerate() {
                let sp = pose_from_uv(*su, *sv, top_left, bottom_left, top_right, bottom_right, lat_o, lon_o);
                let dx = pose.x - sp.x;
                let dy = pose.y - sp.y;
                let d2 = dx * dx + dy * dy;
                if d2 < best_d2 {
                    best_d2 = d2;
                    best_id = sid;
                }
            }

            cells.push(Cell { pose, region_id: best_id });
        }
    }

    cells
}

fn build_regions(
    top_left: (f64, f64),
    bottom_left: (f64, f64),
    top_right: (f64, f64),
    bottom_right: (f64, f64),
    lat_o: f64,
    lon_o: f64,
) -> Vec<RegionPlan> {
    let seeds = region_seed_uvs();
    let all_cells = build_cells(top_left, bottom_left, top_right, bottom_right, lat_o, lon_o);

    let mut regions: Vec<RegionPlan> = seeds
        .iter()
        .enumerate()
        .map(|(id, (row, col, u, v))| RegionPlan {
            id,
            row: *row,
            col: *col,
            cells: Vec::new(),
            center: pose_from_uv(*u, *v, top_left, bottom_left, top_right, bottom_right, lat_o, lon_o),
        })
        .collect();

    for cell in all_cells {
        regions[cell.region_id].cells.push(cell);
    }

    for region in regions.iter_mut() {
        if region.cells.is_empty() {
            continue;
        }

        let mut sum_u = 0.0;
        let mut sum_v = 0.0;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;

        for c in &region.cells {
            sum_u += c.pose.u;
            sum_v += c.pose.v;
            sum_x += c.pose.x;
            sum_y += c.pose.y;
        }

        let n = region.cells.len() as f64;
        let cu = sum_u / n;
        let cv = sum_v / n;
        let _cx = sum_x / n;
        let _cy = sum_y / n;

        region.center = pose_from_uv(
            cu, cv,
            top_left, bottom_left, top_right, bottom_right,
            lat_o, lon_o,
        );
    }

    regions
}

fn dist2_xy(a: (f64, f64), b: (f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    dx * dx + dy * dy
}

fn goto_pose(controller: &OffboardController, pose: &Pose, alt_m: f64, yaw: f64) {
    controller.set_target(pose.x, pose.y, -alt_m, yaw);
    thread::sleep(Duration::from_secs_f64(1.0));
}

fn climb_from_pose(controller: &OffboardController, pose: &Pose, alt_m: f64) {
    controller.set_target(pose.x, pose.y, -alt_m, 0.0);
    thread::sleep(Duration::from_secs(5));
}

fn move_between(
    controller: &OffboardController,
    from: &Pose,
    to: &Pose,
    alt_m: f64,
    step_m: f64,
) {
    let dx = to.x - from.x;
    let dy = to.y - from.y;
    let dist = (dx * dx + dy * dy).sqrt();

    if dist < 1e-6 {
        controller.set_target(to.x, to.y, -alt_m, 0.0);
        thread::sleep(Duration::from_millis(PUBLISH_PERIOD_MS));
        return;
    }

    let yaw = dy.atan2(dx);
    let steps = ((dist / step_m).ceil() as usize).max(2);
    let sleep_s = (step_m / SPEED_MPS).max((PUBLISH_PERIOD_MS as f64) / 1000.0);

    for i in 1..=steps {
        let t = i as f64 / steps as f64;
        let x = lerp(from.x, to.x, t);
        let y = lerp(from.y, to.y, t);
        controller.set_target(x, y, -alt_m, yaw);
        thread::sleep(Duration::from_secs_f64(sleep_s));
    }
}

fn plan_region_route(cells: &[Cell], start_xy: (f64, f64)) -> Vec<usize> {
    if cells.is_empty() {
        return Vec::new();
    }

    let start_idx = (0..cells.len())
        .min_by(|&a, &b| {
            let da = dist2_xy((cells[a].pose.x, cells[a].pose.y), start_xy);
            let db = dist2_xy((cells[b].pose.x, cells[b].pose.y), start_xy);
            da.partial_cmp(&db).unwrap_or(Ordering::Equal)
        })
        .unwrap();

    let mut visited = vec![false; cells.len()];
    let mut pheromone = vec![1.0_f64; cells.len()];
    let mut route = Vec::with_capacity(cells.len());

    route.push(start_idx);
    visited[start_idx] = true;
    let mut current = start_idx;

    while route.len() < cells.len() {
        let candidates: Vec<usize> = (0..cells.len()).filter(|&i| !visited[i]).collect();

        let chosen = candidates
            .into_iter()
            .max_by(|&a, &b| {
                let sa = candidate_score(a, current, cells, &visited, &pheromone);
                let sb = candidate_score(b, current, cells, &visited, &pheromone);
                sa.partial_cmp(&sb).unwrap_or(Ordering::Equal)
            })
            .unwrap();

        route.push(chosen);
        visited[chosen] = true;

        for p in pheromone.iter_mut() {
            *p *= 0.995;
        }
        pheromone[chosen] += 0.8;

        current = chosen;
    }

    route
}

fn candidate_score(
    candidate: usize,
    current: usize,
    cells: &[Cell],
    visited: &[bool],
    pheromone: &[f64],
) -> f64 {
    let cur = &cells[current].pose;
    let cand = &cells[candidate].pose;

    let dx = cand.x - cur.x;
    let dy = cand.y - cur.y;
    let dist = (dx * dx + dy * dy).sqrt();

    let unvisited_local = cells
        .iter()
        .enumerate()
        .filter(|(i, c)| !visited[*i] && c.region_id == cells[candidate].region_id)
        .count() as f64;

    1.5 * unvisited_local + 1.0 * pheromone[candidate] + 1.2 * (1.0 / (0.5 + dist))
}

fn follow_route(
    controller: &OffboardController,
    route: &[usize],
    cells: &[Cell],
    alt_m: f64,
) -> Option<Pose> {
    if route.is_empty() {
        return None;
    }

    let mut current = cells[route[0]].pose;
    goto_pose(controller, &current, alt_m, 0.0);

    for w in route.windows(2) {
        let a = cells[w[0]].pose;
        let b = cells[w[1]].pose;
        move_between(controller, &a, &b, alt_m, STEP_M);
        current = b;
    }

    Some(current)
}

fn order_regions_by_nearest_center(mut regions: Vec<RegionPlan>, start: Pose) -> Vec<RegionPlan> {
    let mut ordered = Vec::with_capacity(regions.len());
    let mut current = start;

    while !regions.is_empty() {
        let idx = (0..regions.len())
            .min_by(|&a, &b| {
                let da = dist2_xy((regions[a].center.x, regions[a].center.y), (current.x, current.y));
                let db = dist2_xy((regions[b].center.x, regions[b].center.y), (current.x, current.y));
                da.partial_cmp(&db).unwrap_or(Ordering::Equal)
            })
            .unwrap();

        let next = regions.swap_remove(idx);
        current = next.center;
        ordered.push(next);
    }

    ordered
}

fn assign_parallel_sets(regions: Vec<RegionPlan>) -> (Vec<RegionPlan>, Vec<RegionPlan>) {
    // Checkerboard split in seed space: adjacent regions go to different drones.
    let mut d1 = Vec::new();
    let mut d2 = Vec::new();

    for r in regions {
        if (r.row + r.col) % 2 == 0 {
            d1.push(r);
        } else {
            d2.push(r);
        }
    }

    (d1, d2)
}

fn cover_region_sequence(
    drone_name: &str,
    controller: Arc<OffboardController>,
    arm: Arc<ArmDrone>,
    regions: Vec<RegionPlan>,
    start_delay_ms: u64,
    cruise_alt: f64,
    origin_pose: Pose,
) -> Pose {
    thread::sleep(Duration::from_millis(start_delay_ms));

    if regions.is_empty() {
        return origin_pose;
    }

    let regions = order_regions_by_nearest_center(regions, origin_pose);
    let mut current_pose = origin_pose;
    let mut airborne = false;

    for (i, region) in regions.iter().enumerate() {
        println!(
            "\n===== {} covering region {} ({} cells) =====",
            drone_name,
            region.id,
            region.cells.len()
        );

        arm.set_offboard();
        arm.arm();

        if !airborne {
            climb_from_pose(&controller, &current_pose, cruise_alt);
            airborne = true;
        } else {
            climb_from_pose(&controller, &current_pose, cruise_alt);
        }

        // Travel to the region center first.
        move_between(&controller, &current_pose, &region.center, cruise_alt, STEP_M);
        current_pose = region.center;

        // Cover the region from its center-side entry.
        let route = plan_region_route(&region.cells, (current_pose.x, current_pose.y));
        if let Some(last) = follow_route(&controller, &route, &region.cells, cruise_alt) {
            current_pose = last;
        }

        // Return to the region center and land there.
        move_between(&controller, &current_pose, &region.center, cruise_alt, STEP_M);
        current_pose = region.center;

        println!(
            "{} landing at region {} center: ({:.6}, {:.6})",
            drone_name, region.id, region.center.lat, region.center.lon
        );

        arm.land();
        thread::sleep(Duration::from_secs(8));
        arm.disarm();
        airborne = false;

        // small deconfliction pause between region swaps
        if i + 1 < regions.len() {
            thread::sleep(Duration::from_millis(500));
        }
    }

    current_pose
}

fn return_home(
    drone_name: &str,
    controller: Arc<OffboardController>,
    arm: Arc<ArmDrone>,
    current_pose: Pose,
    origin_pose: Pose,
    cruise_alt: f64,
) {
    println!("\n===== {} returning to origin pose =====", drone_name);

    arm.set_offboard();
    arm.arm();
    climb_from_pose(&controller, &current_pose, cruise_alt);

    move_between(&controller, &current_pose, &origin_pose, cruise_alt, STEP_M);

    println!(
        "{} landing at origin pose: ({:.6}, {:.6})",
        drone_name, origin_pose.lat, origin_pose.lon
    );

    arm.land();
    thread::sleep(Duration::from_secs(8));
    arm.disarm();
}

/// public mission entrypoint
pub fn run_mission(
    _node: &Node,
    ctrl1: Arc<OffboardController>,
    ctrl2: Arc<OffboardController>,
    arm1: Arc<ArmDrone>,
    arm2: Arc<ArmDrone>,
) -> Result<(), Box<dyn Error>> {
    println!("MISSION START (parallel Voronoi-like coverage, hard deconfliction)");

    let top_left = GLOBAL_CORNERS[0];
    let bottom_left = GLOBAL_CORNERS[1];
    let top_right = GLOBAL_CORNERS[2];
    let bottom_right = GLOBAL_CORNERS[3];

    let lat_o = top_left.0; // <--*
    let lon_o = top_left.1; // <--*

    let origin_pose = pose_from_uv(
        1.0, 0.0,
        top_left, bottom_left, top_right, bottom_right,
        lat_o, lon_o,
    );

    let regions = build_regions(top_left, bottom_left, top_right, bottom_right, lat_o, lon_o);
    println!("Built {} regions", regions.len());

    let (d1_regions, d2_regions) = assign_parallel_sets(regions);

    println!(
        "Assigned regions: px4_1 = {}, px4_2 = {}",
        d1_regions.len(),
        d2_regions.len()
    );

    // Parallel execution with checkerboard deconfliction and altitude separation.
    let c1 = ctrl1.clone();
    let a1 = arm1.clone();
    let o1 = origin_pose;
    let h1 = thread::spawn(move || {
        cover_region_sequence("px4_1", c1, a1, d1_regions, 0, BASE_ALT, o1)
    });

    let c2 = ctrl2.clone();
    let a2 = arm2.clone();
    let o2 = origin_pose;
    let h2 = thread::spawn(move || {
        // small initial delay so both do not leave the origin at the same instant
        cover_region_sequence("px4_2", c2, a2, d2_regions, 2500, DRONE2_ALT, o2)
    });

    let pose1 = h1.join().unwrap_or(origin_pose);
    let pose2 = h2.join().unwrap_or(origin_pose);

    // Bring them back one by one.
    return_home("px4_1", ctrl1.clone(), arm1.clone(), pose1, origin_pose, BASE_ALT);
    return_home("px4_2", ctrl2.clone(), arm2.clone(), pose2, origin_pose, DRONE2_ALT);

    println!("Mission COMPLETE");
    Ok(())
}


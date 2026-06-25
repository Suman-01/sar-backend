use sensor_msgs::LidarScan;
use crate::state::CircleObstacle;

#[derive(Clone, Copy, Debug)]
pub struct Pt {
    x: f64,
    y: f64,
}

fn polar_to_local(r: f64, angle: f64) -> Pt {
    Pt {
        x: r * angle.cos(),
        y: r * angle.sin(),
    }
}

pub fn scan_to_obstacles(
    scan: &LidarScan,
    yaw: f64,
    cluster_gap: f64,
    min_points: usize,
    inflate_m: f64, // Inflation radius in meters
) -> Vec<CircleObstacle> {
    let mut points: Vec<Pt> = Vec::new();
    let mut ang = scan.angle_min as f64;

    for &range in &scan.ranges.iter() { 
        let r = range as f64;
        if r.is_finite() && r > scan.range_min as f64 && r < scan.range_max as f64 {
            let local_pt = polar_to_local(r, ang + yaw);
            points.push(local_pt);
        }
        ang += scan.angle_increment as f64;
    }

    if points.is_empty() {
        return Vec::new();
    }

    let mut clusters: Vec<Vec<Pt>> = Vec::new();
    let curr: Vec<Pt> = vec![points[0]];

    for i in 1..points.len() {
        let a = points[i - 1];
        let b = points[i];
        let dist = ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt();

        // If the distance between points is less than the cluster gap, add to current cluster
        if dist < cluster_gap {
            curr.push(b);
        } 
        else { // Otherwise, start a new cluster
            if curr.len() >= min_points {
                clusters.push(curr.clone());
            }
            curr.clear();
            curr.push(b); 
        }
    }

    // last cluster
    if curr.len() >= min_points {
        clusters.push(curr);
    }

    let mut out = Vec::new();

    for cl in clusters {
        let mut cx = 0.0;
        let mut cy = 0.0;

        for p in &cl {
            cx += p.x;
            cy += p.y;
        }
        cx /= cl.len() as f64;
        cy /= cl.len() as f64;

        let mut max_rad = 0.0;
        for p in &cl {
            let dist = ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt();
            if dist > max_rad {
                max_rad = dist;
            }
        }
        out.push(CircleObstacle {
                n: cx,
                e: cy,
                radius: max_rad + inflate_m,
        });
    }
    out
}

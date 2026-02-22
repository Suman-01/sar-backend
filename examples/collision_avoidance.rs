use sensor_msgs::msg::LaserScan;
use std::sync::{Arc, Mutex};


const SAFE_DIST: f32 = 3.0;
const REPULSE_GAIN: f32 = 1.5;

pub struct  CollisionAvoidance {
    pub scan: Arc<Mutex<Option<LaserScan>>>,
}

impl CollisionAvoidance {
    pub fn new() -> Self {
        Self {
            scan: Arc::new(Mutex::new(None)),
        }
    }

    pub fn update_scan(&self, msg: LaserScan) {
        *self.scan.lock().unwrap() = Some(msg);
    }

    pub fn avoidance_vec(&self) -> (f64, f64) {
        let lock = self.scan.lock().unwrap();

        if lock.is_none() {
            return (0.0, 0.0);
        }

        let scna = lock.as_ref().unwrap();

        let mut ax = 0.0;
        let mut ay = 0.0;

        for (i, r) in scan.ranges.iter().enumerate() {
            if !r.is_finite() || *r > SAFE_DIST {
                continue;
            }

            let angle = scan.angle_min + i as f32 * scan.angle_increment;

            let strength = REPULSIVE_GAIN * (SAFE_DIST - r) / SAFE_DIST;

            ax -= strength as f64 * angle.cos() as f64;
            ay -= strength as f64 * angle.sin() as f64;
        }

        (ax, ay)
    }
}
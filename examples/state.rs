use std::sync::{Arc, Mutex};

#[derive(Clone, Copy, Debug, Default)]
pub struct Pose2D {
    pub n: f64,
    pub e: f64,
    pub d: f64,
    pub yaw: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Twist2D {
    pub vn: f64,
    pub ve: f64,
    pub vd: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DroneState {
    pub pose: Pose2D,
    pub vel: Twist2D,
    pub armed: bool,
    pub offboard: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct CircleObstacle {
    pub n: f64,
    pub e: f64,
    pub radius: f64,
}

pub type SharedDroneState = Arc<Mutex<DroneState>>;
pub type SharedObstacles = Arc<Mutex<Vec<CircleObstacle>>>;
// Only for static obstacles for now, dynamic obstacles can be added later if needed
use crate::math::Vec3;

pub enum ObstacleShape {
    Point,
    Sphere { radius: f64 },
    Cylinder { radius: f64, height: f64 },
}

pub struct Obstacle {
    pub position: Vec3,
    pub velocity: Vec3,
    pub shape: ObstacleShape,
}

impl Obstacle {
    pub fn new(position: Vec3, shape: ObstacleShape) -> Self {
        Self {
            position,
            velocity: Vec3::zero(),
            shape,
        }
    }

    pub fn safety_rad(&self) -> f64 {
        match self.shape {
            ObstacleShape::Sphere { radius } => radius,
            ObstacleShape::Cylinder { radius, .. } => radius,
        }
    }
}


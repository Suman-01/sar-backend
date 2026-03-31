use crate::math::Vec3;
use crate::model::Obstacle;

pub struct LidarSample {
    pub angle_rad: f64,
    pub range_m: f64,
}

// XY plane Lidar
// obstacles are spheres
// static obstacles
pub fn extract_obstacles(
    samples: &[LidarSample],
    obstacle_radius: f64,
    min_range: f64,
    max_range: f64,
) -> Vec<Obstacle> {
    let mut obstacles = Vec::with_capacity(samples.len());  // <--*

    for s in samples{
        if !s.range_m.is_finite() {
            continue;
        }

        if s.range_m < min_range || s.range_m > max_range {
            continue;
        }

        let x = s.range_m * s.angle_rad.cos();
        let y = s.range_m * s.angle_rad.sin();

        obstacles.push(Obstacle::new(
            Vec3::new(x, y, 0.0),
            obstacle_radius
        ));
    }

    obstacles

}
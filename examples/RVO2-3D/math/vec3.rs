use crate::math::utils::{EPSILON};

pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    // new vector
    pub const fn new(x: f64,y: f64, z: f64) -> Self {
        Self{ x, y, z }
    }

    // zero vector
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    // unit vectors
    pub const fn unit_x() -> Self {
        Self::new(1.0, 0.0, 0.0)
    }

    pub const fn unit_y() -> Self {
        Self::new(0.0, 1.0, 0.0)
    }

    pub const fn unit_z() -> Self {
        Self::new(0.0, 0.0, 1.0)
    }

    // mul vec by scalar k
    pub fn mul(&self, k: f64) -> Self {
        Vec3 {
            x: self.x * k,
            y: self.y * k,
            z: self.z * k,
        }
    }

    // div vec by scalar k
    pub fn div(&self, k: f64) -> Self {
        Vec3 {
            x: self.x / k,
            y: self.y / k,
            z: self.z / k,
        }
    }

    // dot product
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    // cross product (self: (x1, y1, z1) x other: (x2, y2, z2) = (y1*z2 - z1*y2, z1*x2 - x1*z2, x1*y2 - y1*x2))
    pub fn cross(self, other: Self) -> Self {
        Vec3 {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn norm_sq(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm_sq().sqrt()
    }

    pub fn normalized(self) -> f64 {
        let n = self.norm_sq();
        if n <= EPSILON {
            Self::zero()
        } else {
            self / n
        }
    }

    pub fn distance_sq(self, other: Self) -> f64 {
        (self - other).norm_sq()
    }

    pub fn distance(self, other: Self) -> f64 {
        self.distance_sq(other).sqrt()
    }

    // gives a velocity vector with the same direction as self but with a magnitude of at most max_val
    pub fn clamp_magnitude(self, max_val: f64) -> Self {
        let n = self.norm();
        if n > max_val && n > EPSILON {
            self * max_val / n 
        } else {
            self
        }
    }



    

    






    
}
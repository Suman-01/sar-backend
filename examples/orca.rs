// orca.rs
use std::ops::{Add, Sub, Mul}; 
use rand::rand_core::block;
use stc::f64::consts::PI;
use crate::state::{DroneState, CircleObstacle};

#[derive(Clone, Copy)]
pub struct Vec2 {
    pub n: f64,
    pub e: f64,
}

impl Vec2 {
    pub fn new(n: f64, e: f64) -> Self {
        Self { n, e }
    }

    pub fn zero() -> Self {
        Self { n: 0.0, e: 0.0 }
    } 

    pub fn norm(self) -> f64 {
        (self.n * self.n + self.e * self.e).sqrt()
    }

    pub fn normalized(self) -> Self {
        let s = self.norm();
        if s < 1e-9 {
            Self::zero()
        } else {
            Self::new(self.n / s, self.e / s)
        }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.n * other.n + self.e * other.e
    }

    // rotate 90 degrees left 
    pub fn p_left(self) -> Self {
        Self::new(self.e, -self.n)
    }
    // rotate 90 degrees right
    pub fn p_right(self) -> Self {
        Self::new(-self.e, self.n)
    }

    pub fn clamp(self, max_val: f64) -> Self {
        let s = self.norm();
        if s <= max_val || s < 1e-9 {
            self 
        } else {
            self * (max_val / s)
        }
    }

}

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.n + rhs.n, self.e + rhs.e)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.n - rhs.n, self.e - rhs.e)
    }
}

impl Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.n * rhs, self.e * rhs)
    }
}

pub struct OrcaConfig {
    pub dt: f64,
    pub t_h: f64,
    pub max_speed: f64,
    pub safety_margin: f64,
    pub lookahead: f64,
    pub stuck_eps: f64,
    pub stuck_limit: u32,
    pub angle_samples: usize,
    pub speed_samples: usize,
}

pub struct OrcaPlanner {
    pub cfg: OrcaConfig,
}

impl OrcaPlanner {
    pub fn new(cfg: OrcaConfig) -> Self {
        Self { cfg }
    }

    pub fn yaw_from_vel(v: Vec2) -> f64 {
        if v.norm() < 1e-9 {
            0.0
        } else {
            v.e.atan2(v.n)
        }
    }

    fn goal_blocked(&self, pos: Vec2, goal_dir: Vec2, obstacles: [&CircleObstacle]) -> bool {
        for o in obstacles {
            let rel = Vec2::new(o.n, o.e) - pos;
            let proj = rel.dot(goal_dir);   // vel magnitude along goal direction
            if proj <= 0.0 || proj > self.cfg.lookahead {
                continue;
            }
            let closest = pos + goal_dir * proj; 
            let lateral = (Vec2::new(o.n, o.e) - closest).norm(); // perp dis b/w obs cen and goal line
            let inflated = o.radius + self.cfg.safety_margin;
            if lateral <= inflated {
                return true;
            }
        }
        false
    }

    fn safe_for_horizon(
        &self, 
        pos: Vec2,
        cand_vel: Vec2,
        others: &[DroneState],
        obstacles: [&CircleObstacle],
    ) -> bool {
        let steps = (self.cfg.t_h / self.cfg.dt).ceil().max(1.0) as usize; // max of steps or 1.0

        for i in 1..=steps {
            let t = i as f64 * self.cfg.dt;
            let p = pos + cand_vel * t;

            // obstacles
            for o in obstacles {
                let d = (p - Vec2::new(o.n, o.e)).norm();
                if d <= o.radius + self.cfg.safety_margin {
                    return false;
                }
            }

            // others (to be implemented)

        }

        true
    }

    fn cost(
        &self, 
        pos: Vec2,
        cand_vel: Vec2,
        pref_vel: Vec2,
        goal_dir: Vec2,
        blocked: bool,
        me: &DroneState,
    )  -> f64 {
        let mut c = 0.0;

        c += (cand_vel - pref_vel).norm() * 1.0;  
        c += (cand_vel - Vec2::new(me.vel.vn, me.vel.ve)).norm() * 0.2;
        c += -cand_vel.dot(goal_dir) * 1.5; 

        if blocked {
            let lateral = goal_dir.p_left().dot(cand_vel); 
            let side_sign  = if lateral > 0.0 { 1.0 } else { -1.0 };    
            if side_sign < 0.0 {
                c += lateral.abs() * 1.5;
            } else {
                c += lateral.abs() * 0.5;
            }
        }

        let _ = pos;
        c

    }

    pub fn step(
        &self,
        me: &DroneState, 
        others: &[DroneState],
        obstacles: [&CircleObstacle], 
        goal_n: f64,
        goal_e: f64,
    ) -> Vec2 {
        let pose = Vec2::new(me.pose.n, me.pose.e);
        let goal = Vec2::new(goal_n, goal_e);
        let to_goal = goal - pose;
        let goal_dist = to_goal.norm();
        if goal_dist < 1e-6 {
            return Vec2::zero();
        }

        let goal_dir = to_goal.normalized();
        let pref = goal_dir * self.cfg.max_speed;

        let blocked = self.goal_blocked(pose, goal_dir, obstacles);

        // if goal is blocked -> then try to turn left or right and go around
        // can scale down the velocity relative to the closeness of the obstacle
        // why are we calculating the left and right directions if we are not using them in the candidate generation? 
        // 
        if blocked {
            let left = (goal_dir * 0.20 + goal_dir.p_left() * 0.80).normalized() * self.cfg.max_speed;
            let right = (goal_dir * 0.20 + goal_dir.p_right() * 0.80).normalized() * self.cfg.max_speed;

            let left_ok = self.safe_for_horizon(pose, left, others, obstacles);
            let right_ok = self.safe_for_horizon(pose, right, others, obstacles);

            if left_ok && !right_ok {
                me.vel.vn = left.n;
                me.vel.ve = left.e;
                me.pose.yaw = Self::yaw_from_vel(left);
                return left;
            }

            if !left_ok && right_ok {
                me.vel.vn = right.n;
                me.vel.ve = right.e;
                me.pose.yaw = Self::yaw_from_vel(right);
                return right;
            }

            if left_ok && right_ok {
                // choose the one closer to the goal direction
                let left_sc = left.dot(pref);
                let right_sc = right.dot(pref);
                if left_sc > right_sc {
                    me.vel.vn = left.n;
                    me.vel.ve = left.e;
                    me.pose.yaw = Self::yaw_from_vel(left);
                    return left;
                } else {
                    me.vel.vn = right.n;    
                    me.vel.ve = right.e;
                    me.pose.yaw = Self::yaw_from_vel(right);
                    return right;
                }
            }
        }

        let mut candidates: Vec<Vec2> = Vec::new();
        candidates.push(pref);
        candidates.push((goal_dir * 0.15 + goal_dir.p_left() * 0.90).normalized() * self.cfg.max_speed);
        candidates.push((goal_dir * 0.15 + goal_dir.p_right() * 0.90).normalized() * self.cfg.max_speed);

        
        let ang_n = self.cfg.angle_samples.max(16);
        let sp_n = self.cfg.speed_samples.max(4);
        
        // if sp_n is 4 and ang_n in 16, we will have total 1 (pref) + 2 (left/right) + 4*16 (candidates) = 67 candidates to evaluate.
        for i in 0..sp_n {
            let s = self.cfg.max_speed * (i as f64 + 1.0) / sp_n as f64;    // linearly spaced speeds from max_speed/sp_n to max_speed
            for j in 0..ang_n {
                let a = -PI + 2.0 * PI * (j as f64) / ang_n as f64; // linearly spaced angles from -pi to pi
                candidates.push(Vec2::new(a.cos() * s, a.sin() * s));   // add candidate in the direction of angle a with speed s
            }
        }

        let mut best = Vec2::zero();
        let mut best_cost = f64::INFINITY;

        for c in candidates {
            if !self.safe_for_horizon(pose, c, others, obstacles) {
                continue;
            }

            let cost = self.cost(pose, c, pref, goal_dir, blocked, me);
            if cost < best_cost {
                best_cost = cost;
                best = c;
            }
        }

        // If nothing is safe, stop briefly and let the next cycle try again.
        if !best_cost.is_finite() {
            best = Vec2::zero();
        }

        me.vel.vn = best.n;
        me.vel.ve = best.e;
        me.pose.yaw = Self::yaw_from_vel(best);
        best

    }




}




use crate::math::Vec3;

pub struct Agent {
    pub id: usize,
    pub position: Vec3,
    pub velocity: Vec3,
    pub pref_vel: Vec3,
    pub goal: Vec3,
    pub radius: f64,
    pub max_speed: f64,
    pub time_h: f64,
    pub neigh_dist: f64,
    pub max_neigh: usize,
}

impl Agent {
    pub fn new(id: usize, position: Vec3, goal: Vec3) -> Self {
        Self {
            id,
            position,
            velocity: Vec3::zero(),
            pref_vel: Vec3::zero(),
            goal,
            radius: 0.5,        //manual
            max_speed: 3.0,     //manual
            time_h: 5.0,        //manual
            neigh_dist: 10.0,   //manual
            max_neigh: 10,      //manual
        }
    }

    pub fn upd_pref_vel(&mut self) {
        let goal_dis = self.goal - self.position;
        self.pref_vel = goal_dis.clamp_magnitude(self.max_speed); // we clamp 
    }

    pub fn reached_goal(&self, tolerance: f64) -> bool{
        self.position.distance(self.goal) <= tolerance
    }
}
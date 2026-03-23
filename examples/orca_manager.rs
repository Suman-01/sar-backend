// ORCA for Collision Avoidance

use std::borrow::Cow;
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::{Duration, Instant};

use dodgy_3d::{Agent, Vec3, AvoidanceOptions};

use crate::offboard::OffboardController;

pub struct OrcaManager {
    // Everything stored in vectors with index equal to the agent number(agent_ix)
    agents: Arc<Mutex<Vec<Cow<'static, Agent>>>>,   // store agent params 
    goal_points: Arc<Mutex<Vec<Vec3>>>,   // next goal point for each agent
    agent_max_speeds: Arc<Mutex<Vec<f32>>>,   // store max allowed speed per agent
    agent_time_horizons: Arc<Mutex<Vec<f32>>>,   // store the time horizon per agent
    agent_radii: Arc<Mutex<Vec<f32>>>,   // store radius of each drone
    controllers: Vec<Arc<OffboardController>>,  // n_agents = controllers.len()
    running: Arc<AtomicBool>,   // running status
    dt: f32,    // time delta (seconds per cycle)
}

impl OrcaManager {
    // hz <-- orca loop frq (cycles per second)
    pub fn new(controllers: Vec<Arc<OffboardController>>, hz: f32) -> Self {
        let n = controllers.len();
        let mut agents_vec:Vec<Cow<'static, Agent>> = Vec::with_capacity(n);
        let mut goals: Vec<Vec3> = Vec::with_capacity(n);
        let mut max_speeds: Vec<f32> = Vec::with_capacity(n);
        let mut t_h: Vec<f32> = Vec::with_capacity(n);
        let mut radii: Vec<f32> = Vec::with_capacity(n);

        for _ in 0..n {
            agents_vec.push(Cow::Owned(Agent {
                position: Vec3::ZERO,
                velocity: Vec3::ZERO,
                radius: 1.0,    // initialized with agent radius 1m (change value if required)
                avoidance_responsibility: 1.0,
            }));

            goals.push(Vec3::ZERO);
            max_speeds.push(2.0);   // initialized with max vel 2.0 m/s per agent (change value if required)
            t_h.push(3.0);  // initialized with 3s horizon (change value if required)
            radii.push(1.0); // initialized with agent radius 1m (change value if required)
        }

        OrcaManager { 
            agents: Arc::new(Mutex::new(agents_vec)), 
            goal_points: Arc::new(Mutex::new(goals)), 
            agent_max_speeds: Arc::new(Mutex::new(max_speeds)), 
            agent_time_horizons: Arc::new(Mutex::new(t_h)), 
            agent_radii: Arc::new(Mutex::new(radii)), 
            controllers,
            running: Arc::new(AtomicBool::new(false)),
            dt: 1.0 / hz.max(1.0),
        }
    }

    // set initial pos. (n_m, e_m, alt_m) for all agents
    pub fn set_pos(&self, pos: &[(f32, f32, f32)]) {
        let mut agents = self.agents.lock().unwrap(); // lock it
        for (i, &(n, e, alt)) in pos.iter().enumerate().take(agents.len()) {    // enumerate through each i, (pos) pair
            if let Some(agent) = agents[i].to_mut() {   // select agent
                agent.position = Vec3::new(n, e, alt);  // set the pos
                agent.velocity = Vec3::ZERO;    // vel = 0
            }
        }
    }

    // Goal per agent
    pub fn set_goal(&self, agent_ix: usize, goal: (f32, f32, f32)) {
        let mut goals = self.goal_points.lock().unwrap();
        goals[agent_ix] = Vec3::new(goal.0, goal.1, goal.2);
    }

    // Set (radius, max_speed, time_horizon) per agent
    pub fn set_params(&self, agent_ix: usize, radius: f32, max_speed: f32, time_horizon: f32) {
        {
            let mut radii = self.agent_radii.lock().unwrap();
            radii[agent_ix] = radius;
        }

        {
            let mut max_speeds = self.agent_max_speeds.lock().unwrap();
            max_speeds[agent_ix] = max_speed;
        }

        {
            let mut t_h = self.agent_time_horizons.lock().unwrap();
            t_h[agent_ix] = time_horizon;
        }
    }

    // spawn detached thread -> run the orca loop -> compute CA vels -> update pos by dt -> publish setpoint
    pub fn start_orca(&self) {
        if self.running.swap(true, Ordering::SeqCst) {
            // running
            return;
        }

        // if not running
        let agents = Arc::clone(&self.agents);
        let goals = Arc::clone(&self.goal_points);
        let speeds = Arc::clone(&self.agent_max_speeds);
        let t_h = Arc::clone(&self.agent_time_horizons);
        let radii = Arc::clone(&self.agent_radii);
        let controllers = self.controllers.clone();
        let running = Arc::clone(&self.running);
        let dt = self.dt;

        thread::spawn(move || {
            while running.load(Ordering::SeqCst) {  // <--*
                let loop_start = Instant::now();

                // compute CA vels
                let new_velocities: Vec<Vec3> = {
                    let agents_guard = agents.lock().unwrap();
                    let goals_guard = goals.lock().unwrap();
                    let speeds_guard = speeds.lock().unwrap();
                    let horizons_guard = t_h.lock().unwrap();




                }


            }
        });
    }
}
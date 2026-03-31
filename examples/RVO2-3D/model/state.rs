use crate::model::{Agent, Obstacle};

#[derive(Clone, Debug, Default)]
pub struct PlanningState {
    pub agents: Vec<Agent>,
    pub obstacles: Vec<Obstacle>,
    pub time: f64,
}

impl PlanningState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity() -> Self {
        Self {

        }
    }

    pub fn add_agent() {

    }

    pub fn add_obstacle() {
        
    }

    pub fn advance_time() {
        
    }
}




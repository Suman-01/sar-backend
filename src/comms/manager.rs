use std::collections::HashMap;
use crate::comms::comms_metrics::{CommStats, CommsMetrics};

pub struct CommsManager {
    drones: HashMap<u32, CommsMetrics>,
}

impl CommsManager {
    pub fn new_comms_manager() -> Self {
        Self { drones: HashMap::new() }
    }

    pub fn add_drone(&mut self, drone_id: u32) {
        self.drones.insert(drone_id, CommsMetrics::new_comms_metrics(drone_id));
    }

    pub fn remove_drone(&mut self, drone_id: u32) {
        self.drones.remove(&drone_id);
    }

    pub fn get_metrics(&self, drone_id: u32) -> Option<&CommsMetrics> {
        self.drones.get(&drone_id)
    }

    pub fn update_metrics(&mut self, drone_id: u32, latency_ms: f32, packet_loss: f32, is_alive: bool) {
        if let Some(metrics) = self.drones.get_mut(&drone_id) {
            metrics.set_latency_ms(latency_ms);
            metrics.set_packet_loss(packet_loss);
            metrics.set_is_alive(is_alive);
        }
    }

    pub fn update_latency_ms(&mut self, drone_id: u32, latency_ms: f32) {
        if let Some(metrics) = self.drones.get_mut(&drone_id) {
            metrics.set_latency_ms(latency_ms);
        }
    }

    pub fn update_packet_loss(&mut self, drone_id: u32, packet_loss: f32) {
        if let Some(metrics) = self.drones.get_mut(&drone_id) {
            metrics.set_packet_loss(packet_loss);
        }
    }

    pub fn update_is_alive(&mut self, drone_id: u32, is_alive: bool) {
        if let Some(metrics) = self.drones.get_mut(&drone_id) {
            metrics.set_is_alive(is_alive);
        }
    }

    pub fn get_status(&self, drone_id: u32) -> Option<CommStats> {
        self.drones.get(&drone_id).map(|m| m.get_comms_stats())
    }

    pub fn list_drones(&self) -> Vec<u32> {
        self.drones.keys().cloned().collect()
    }
        
}


// Check the udate functions | descrete updation of each metric + all-together update 
// Might be redundant
// Also used setter functions | any viable replacement to the updateion procedure
// Is it runtime intensive?
// Clamp the inputs to update functions 
// Use single funtion for updation of metrics with Options<T> u don't have to make so many functions

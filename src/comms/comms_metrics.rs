#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommStats {
    Good,
    Degraded,
    Lost,
}

#[derive(Debug, Clone)]
pub struct CommsMetrics {
    //drone_id: u32,
    latency_ms: f32,
    packet_loss: f32,
    is_alive: bool,
}

impl CommsMetrics {
    pub fn new_comms_metrics(drone_id: u32) -> Self {
        Self {
            //drone_id,
            latency_ms: 0.0,
            packet_loss: 0.0,
            is_alive: true,
        }
    }
    // Getter Functions ---
    pub fn get_latency_ms(&self) -> f32 {self.latency_ms}
    pub fn get_packet_loss(&self) -> f32 {self.packet_loss}
    pub fn get_is_alive(&self) -> bool {self.is_alive}
    // Getter Functions ---

    // Setter Functions ---
    pub(crate) fn set_latency_ms(&mut self, latency_ms: f32) {self.latency_ms = latency_ms}
    pub(crate) fn set_packet_loss(&mut self, packet_loss: f32) {self.packet_loss = packet_loss}
    pub(crate) fn set_is_alive(&mut self, is_alive: bool) {self.is_alive = is_alive}
    // Setter Functions ---



    pub fn get_comms_stats(&self) -> CommStats {
        if !self.is_alive {
            CommStats::Lost
        } else if self.latency_ms > 500.0 || self.packet_loss > 20.0 {
            CommStats::Degraded
        } else {
            CommStats::Good
        }
    }
}


//Encapsulation needed
//Metrics functions should be here not in manager
//Manager should be have only right to call the functions
//Values are clamped. Will not work in field. Not real too
//No drone id in metrics structure because of manager hashmap


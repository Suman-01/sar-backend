use chrono::{DateTime, Utc};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum EventType {
    TakeOff,
    Landing,
    WaypointReached,
    Detection(String),
    CommsLoss,
    Other(String),
}

#[derive(Debug, Clone)]
pub struct Event {
    pub timestamp: DateTime<Utc>,
    pub drone_id: u32,
    pub event_type: EventType,
    pub latitude: Option<f64>,
    pub longitude: Option<f64>,
}

impl Event {
    pub fn new_event(drone_id: u32, event_type: EventType, latitude: Option<f64>, longitude: Option<f64>) -> Self {
        Self {
            timestamp: Utc::now(),
            drone_id,
            event_type,
            latitude,
            longitude,
        }
    }
}
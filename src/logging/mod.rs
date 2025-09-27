pub mod event;

use crate::logging::event::{Event, EventType};

#[derive(Debug, Clone)]
pub struct DataLoger {
    pub logs: Vec<Event>
}

impl DataLoger {
    pub fn new_dataloger() -> Self {
        Self {
            logs: Vec::new()
        }
    }

    pub fn add_event(&mut self, event: Event) {
        self.logs.push(event);
    }

    pub fn get_drone_events(&self, drone_id: u32) -> Vec<&Event> {
        self.logs.iter().filter(|e| e.drone_id == drone_id).collect()
    }

    pub fn get_event_by_type(&self, event_type: EventType) -> Vec<&Event> {
        self.logs.iter().filter(|e| e.event_type == event_type).collect()
    }
}

//We can create a csv file for the logger too

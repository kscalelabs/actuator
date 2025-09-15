use eyre::Result;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, RwLock};
use std::time::{Duration, SystemTime};
use tracing::{debug, error, info, trace, warn};

use crate::{
    actuator_types::ActuatorConfiguration,
    transport::{Transport, TransportType},
    ActuatorType, ControlCommand, FeedbackFrame,
};

// Store the latest feedback with timestamp
#[derive(Clone, Debug)]
pub struct ActuatorState {
    pub feedback: Option<FeedbackFrame>,
    pub last_feedback: SystemTime,
    pub last_command: SystemTime,
    pub ready: bool,
    pub enabled: bool,
    pub control_config: ControlConfig,
    pub control_command: ControlCommand,
    pub configuration: ActuatorConfiguration,
    pub messages_received: u64,
    pub half_revolutions: i32,
    pub actuator_type: ActuatorType,
}

#[derive(Clone, Debug)]
pub struct ControlConfig {
    pub kp: f32,
    pub kd: f32,
    pub max_torque: Option<f32>,
    pub max_velocity: Option<f32>,
    pub max_current: Option<f32>,
}

pub struct TransportHandler {
    transport: TransportType,
}

pub struct Supervisor {
    actuators: Arc<RwLock<HashMap<u8, ActuatorState>>>,
    transports: Arc<RwLock<HashMap<String, TransportHandler>>>,
    discovered_ids: Arc<RwLock<Vec<u8>>>,
    last_stats_time: SystemTime,
    feedback_timeout: Duration,
}

impl Supervisor {
    pub fn new(feedback_timeout: Duration) -> Result<Self> {
        let supervisor = Self {
            actuators: Arc::new(RwLock::new(HashMap::new())),
            transports: Arc::new(RwLock::new(HashMap::new())),
            discovered_ids: Arc::new(RwLock::new(Vec::new())),
            last_stats_time: SystemTime::now(),
            feedback_timeout,
        };

        Ok(supervisor)
    }

    pub fn add_transport(&self, name: String, transport: TransportType) -> Result<()> {
        info!("Adding transport: {}", name);

        let mut transports = self.transports.write().unwrap();
        transports.insert(name.clone(), TransportHandler { transport });
        info!("Transport {} added successfully", name);

        Ok(())
    }

    pub fn add_actuator(
        &self,
        actuator_id: u8,
        actuator_type: ActuatorType,
        configuration: ActuatorConfiguration,
    ) {
        let state = ActuatorState {
            feedback: None,
            last_feedback: SystemTime::now(),
            last_command: SystemTime::now(),
            ready: false,
            enabled: false,
            control_config: ControlConfig {
                kp: 0.0,
                kd: 0.0,
                max_torque: None,
                max_velocity: None,
                max_current: None,
            },
            control_command: ControlCommand {
                target_angle: 0.0,
                target_velocity: 0.0,
                kp: 0.0,
                kd: 0.0,
                torque: 0.0,
            },
            configuration,
            messages_received: 0,
            half_revolutions: 0,
            actuator_type,
        };

        let mut actuators = self.actuators.write().unwrap();
        actuators.insert(actuator_id, state);

        debug!(
            "Added actuator with ID: {} (type: {:?})",
            actuator_id, actuator_type
        );
    }

    pub fn scan_bus(
        &mut self,
        _host_id: u8,
        transport_name: &str,
        actuator_configs: &[(u8, ActuatorConfiguration)],
    ) -> Result<Vec<u8>> {
        // Get the transport protocol
        let mut transports = self.transports.write().unwrap();
        let transport_handler = transports
            .get_mut(transport_name)
            .ok_or_else(|| eyre::eyre!("Transport not found: {}", transport_name))?;

        {
            let mut discovered = self.discovered_ids.write().unwrap();
            discovered.clear();
        }

        // Clear the buffer
        transport_handler.transport.clear_buffer()?;

        // For now, just add the configured actuators
        for (id, config) in actuator_configs {
            self.add_actuator(*id, config.actuator_type, config.clone());
            let mut discovered = self.discovered_ids.write().unwrap();
            discovered.push(*id);
        }

        let discovered = self.discovered_ids.read().unwrap();
        Ok(discovered.clone())
    }

    pub fn get_actuators_state(&self, actuator_ids: Vec<u8>) -> Vec<ActuatorState> {
        let actuators = self.actuators.read().unwrap();
        let mut states = Vec::new();

        for id in actuator_ids {
            if let Some(state) = actuators.get(&id) {
                states.push(state.clone());
            }
        }

        states
    }

    pub fn command_actuators(&self, commands: Vec<ControlCommand>) -> Vec<bool> {
        let mut results = Vec::new();

        for _command in commands {
            // For now, just return success
            results.push(true);
        }

        results
    }

    pub fn run(&mut self, _interval: Duration) -> Result<()> {
        // Simple synchronous run loop
        loop {
            // Process any incoming messages from transports
            let mut transports = self.transports.write().unwrap();
            for (name, transport_handler) in transports.iter_mut() {
                match transport_handler.transport.recv() {
                    Ok((_id, _data)) => {
                        trace!("Received message from transport: {}", name);
                        // Process the message here
                    }
                    Err(_) => {
                        // No message or error, continue
                    }
                }
            }

            // Sleep for a short time to avoid busy waiting
            std::thread::sleep(Duration::from_millis(10));
        }
    }
}

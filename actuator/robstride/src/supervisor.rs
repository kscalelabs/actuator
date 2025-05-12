use eyre::Result;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, SystemTime};
use tokio::sync::{mpsc, RwLock};
use tokio::time;
use tracing::{debug, error, info, trace, warn};

use crate::{
    actuator::{normalize_value, TypedCommandData, TypedFeedbackData},
    actuator_types::ActuatorConfiguration,
    robstride00::{RobStride00, RobStride00Command, RobStride00Feedback, RobStride00Parameter},
    robstride01::{RobStride01, RobStride01Command, RobStride01Feedback, RobStride01Parameter},
    robstride02::{RobStride02, RobStride02Command, RobStride02Feedback, RobStride02Parameter},
    robstride03::{RobStride03, RobStride03Command, RobStride03Feedback, RobStride03Parameter},
    robstride04::{RobStride04, RobStride04Command, RobStride04Feedback, RobStride04Parameter},
    transport::TransportType,
    Actuator, Command, ControlCommand, FeedbackFrame, Frame, Protocol, TxCommand,
};
use crate::{ActuatorType, FaultFeedback};

// Add the StateUpdate enum at the top of the file
#[derive(Debug)]
enum StateUpdate {
    Feedback(FeedbackFrame),
    ObtainID(u8),
    Fault(FaultFeedback),
}

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
    #[allow(unused)]
    protocol: Protocol,
    tx: mpsc::Sender<TxCommand>,
    #[allow(unused)]
    rx: mpsc::Receiver<TxCommand>,
}

struct ActuatorRecord {
    actuator: Box<dyn Actuator>,
    state: ActuatorState,
}

pub struct Supervisor {
    actuators: Arc<RwLock<HashMap<u8, ActuatorRecord>>>,
    transports: Arc<RwLock<HashMap<String, TransportHandler>>>,
    discovered_ids: Arc<RwLock<Vec<u8>>>,
    last_stats_time: SystemTime,
    feedback_timeout: Duration,
    state_update_tx: mpsc::Sender<StateUpdate>,
}

fn half_revolutions(degrees: f32) -> i32 {
    if degrees < 0.0 {
        (degrees / 180.0).ceil() as i32
    } else {
        (degrees / 180.0).floor() as i32
    }
}

fn normalize_degrees(degrees: f32) -> (f32, i32) {
    let rotations = half_revolutions(degrees);
    let normalized =
        degrees - 360.0 * ((rotations + if rotations >= 0 { 1 } else { -1 }) / 2) as f32;
    (normalized, rotations)
}

fn normalize_radians(radians: f32) -> (f32, i32) {
    let degrees = radians.to_degrees();
    let (normalized_deg, rotations) = normalize_degrees(degrees);
    (normalized_deg.to_radians(), rotations)
}

fn denormalize_degrees(normalized_angle: f32, half_rotations: i32) -> f32 {
    if half_rotations >= 0 {
        normalized_angle + 360.0 * ((half_rotations + 1) / 2) as f32
    } else {
        normalized_angle + 360.0 * (half_rotations / 2) as f32
    }
}

fn denormalize_radians(normalized_radians: f32, half_rotations: i32) -> f32 {
    let normalized_degrees = normalized_radians.to_degrees();
    let original_degrees = denormalize_degrees(normalized_degrees, half_rotations);
    original_degrees.to_radians()
}

impl Supervisor {
    pub fn new(feedback_timeout: Duration) -> Result<Self> {
        let (state_update_tx, mut state_update_rx) = mpsc::channel(32);

        let supervisor = Self {
            actuators: Arc::new(RwLock::new(HashMap::new())),
            transports: Arc::new(RwLock::new(HashMap::new())),
            discovered_ids: Arc::new(RwLock::new(Vec::new())),
            last_stats_time: SystemTime::now(),
            feedback_timeout,
            state_update_tx,
        };

        // Spawn a task to handle state updates asynchronously
        {
            let actuators = supervisor.actuators.clone();
            let discovered_ids = supervisor.discovered_ids.clone();
            tokio::spawn(async move {
                while let Some(update) = state_update_rx.recv().await {
                    match update {
                        StateUpdate::Feedback(feedback) => {
                            let mut actuators_guard = actuators.write().await;
                            if let Some(record) = actuators_guard.get_mut(&feedback.motor_id) {
                                record.state.feedback = Some(feedback.clone());
                                record.state.last_feedback = SystemTime::now();
                                trace!(event = "FeedbackProcessed", motor_id = feedback.motor_id, feedback_details = ?feedback, "Actuator state updated with feedback");
                                record.state.messages_received += 1;
                                if record.state.messages_received >= 5 {
                                    // robstride lol
                                    // wait for 5 messages before marking as ready
                                    record.state.ready = true;
                                }

                                if !record.state.enabled {
                                    // save last pos to move to it on actuator enable
                                    record.state.control_command.target_angle = feedback.angle;
                                }

                                let angle_rad =
                                    RobStride04Feedback::from_feedback_frame(feedback).angle_rad;
                                record.state.half_revolutions = normalize_radians(angle_rad).1;
                            }
                        }
                        StateUpdate::ObtainID(motor_id) => {
                            let mut discovered = discovered_ids.write().await;
                            if !discovered.contains(&motor_id) {
                                discovered.push(motor_id);
                                info!("Discovered new actuator ID: {}", motor_id);
                            }
                        }
                        StateUpdate::Fault(fault) => {
                            warn!("Fault received: {:?}", fault);
                        }
                    }
                }
            });
        }

        Ok(supervisor)
    }

    pub fn clone_controller(&self) -> Self {
        Self {
            actuators: self.actuators.clone(),
            transports: self.transports.clone(),
            discovered_ids: self.discovered_ids.clone(),
            last_stats_time: self.last_stats_time,
            feedback_timeout: self.feedback_timeout,
            state_update_tx: self.state_update_tx.clone(),
        }
    }

    pub async fn add_transport(&self, name: String, transport: TransportType) -> Result<()> {
        info!("Adding transport: {}", name);
        let (tx, mut rx) = mpsc::channel(32);

        let state_update_tx = self.state_update_tx.clone();
        let name_clone = name.clone();
        let name_for_log = name_clone.clone();

        // Create callback for frame processing
        let frame_callback: Arc<dyn Fn(u32, Vec<u8>) + Send + Sync + 'static> = Arc::new(
            move |id: u32, data: Vec<u8>| {
                let cmd = Command::from_can_packet(id, data.clone());
                trace!(
                    "Transport callback received: id={:x}, data={:02x?}, cmd={:?}",
                    id,
                    data,
                    cmd
                );

                if let Ok(cmd_frame) = cmd.to_frame() {
                    match cmd_frame {
                        Frame::Feedback(feedback) => {
                            trace!(event = "FeedbackParsed", raw_can_id = id, motor_id = feedback.motor_id, feedback_details = ?feedback, "Supervisor parsed FeedbackFrame");
                            let _ = state_update_tx.try_send(StateUpdate::Feedback(feedback));
                        }
                        Frame::ObtainID(oid) => {
                            let _ = state_update_tx.try_send(StateUpdate::ObtainID(oid.host_id));
                        }
                        Frame::Fault(fault) => {
                            let _ = state_update_tx.try_send(StateUpdate::Fault(fault));
                        }
                        _ => trace!("received: {:?}", cmd_frame),
                    }
                } else {
                    warn!("Failed to parse frame from command: {:?}", cmd);
                }
            },
        );

        let protocol = Protocol::new(transport.clone(), frame_callback);
        debug!("Created protocol for transport: {}", name);

        // Spawn the transport handling task
        let mut protocol_clone = protocol.clone();
        tokio::spawn(async move {
            info!("Starting transport handling task for {}", name_clone);
            loop {
                tokio::select! {
                    // Handle incoming messages
                    recv_result = protocol_clone.recv() => {
                        match recv_result {
                            Ok(_) => trace!("Received message successfully"),
                            Err(e) => {
                                error!("Transport receiver error: {}", e);
                                break;
                            }
                        }
                    }
                    // Handle outgoing messages
                    Some(cmd) = rx.recv() => {
                        trace!("Processing outgoing command: {:?}", cmd);
                        match cmd {
                            TxCommand::Send { id, data } => {
                                trace!(event = "CommandSentToTransport", can_id = id, data_len = data.len(), data_bytes = ?data, "Supervisor dispatching command to transport");
                                if let Err(e) = protocol_clone.send(id, &data).await {
                                    error!("Transport sender error: {}", e);
                                }
                            }
                        }
                    }
                }
            }
        });

        let mut transports = self.transports.write().await;
        transports.insert(
            name,
            TransportHandler {
                protocol,
                tx: tx.clone(),
                rx: mpsc::channel(32).1,
            },
        );
        info!("Transport {} added successfully", name_for_log);

        Ok(())
    }

    pub async fn get_transport_tx(&self, transport_name: &str) -> Result<mpsc::Sender<TxCommand>> {
        let transports = self.transports.read().await;
        let transport = transports
            .get(transport_name)
            .ok_or_else(|| eyre::eyre!("Transport not found: {}", transport_name))?;
        Ok(transport.tx.clone())
    }

    pub async fn add_actuator(
        &self,
        actuator: Box<dyn Actuator>,
        configuration: ActuatorConfiguration,
    ) {
        let actuator_id = actuator.id();
        let actuator_type = actuator.actuator_type();

        let record = ActuatorRecord {
            actuator,
            state: ActuatorState {
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
            },
        };

        let mut actuators = self.actuators.write().await;
        actuators.insert(actuator_id, record);

        debug!(
            "Added actuator with ID: {} (type: {:?})",
            actuator_id, actuator_type
        );
    }

    pub async fn scan_bus(
        &mut self,
        host_id: u8,
        transport_name: &str,
        actuator_configs: &[(u8, ActuatorConfiguration)],
    ) -> Result<Vec<u8>> {
        let transport_tx = self.get_transport_tx(transport_name).await?;

        {
            let mut discovered = self.discovered_ids.write().await;
            discovered.clear();
        }

        // Send get_uuid to all possible IDs
        for id in 0..=0xFF {
            // Use desired type if specified, otherwise default to RobStride04
            let actuator: Box<dyn Actuator> = match actuator_configs
                .iter()
                .find(|(desired_id, _)| *desired_id == id)
            {
                Some((_, config)) => match config.actuator_type {
                    ActuatorType::RobStride00 => {
                        Box::new(RobStride00::new(id, host_id, transport_tx.clone()))
                    }
                    ActuatorType::RobStride01 => {
                        Box::new(RobStride01::new(id, host_id, transport_tx.clone()))
                    }
                    ActuatorType::RobStride02 => {
                        Box::new(RobStride02::new(id, host_id, transport_tx.clone()))
                    }
                    ActuatorType::RobStride03 => {
                        Box::new(RobStride03::new(id, host_id, transport_tx.clone()))
                    }
                    ActuatorType::RobStride04 => {
                        Box::new(RobStride04::new(id, host_id, transport_tx.clone()))
                    }
                },
                None => Box::new(RobStride04::new(id, host_id, transport_tx.clone())),
            };
            let _ = actuator.get_uuid().await;
            time::sleep(Duration::from_millis(1)).await;
        }

        let timeout = Duration::from_millis(100);
        let scan_end = SystemTime::now() + timeout;

        while SystemTime::now() < scan_end {
            // Get a snapshot of discovered IDs
            let discovered_ids = {
                let discovered = self.discovered_ids.read().await;
                discovered.clone()
            };

            // Process any new IDs
            for id in discovered_ids {
                let mut actuators = self.actuators.write().await;
                if !actuators.contains_key(&id) {
                    let (actuator, configuration): (Box<dyn Actuator>, ActuatorConfiguration) =
                        match actuator_configs
                            .iter()
                            .find(|(desired_id, _)| *desired_id == id)
                        {
                            Some((_, config)) => match config.actuator_type {
                                ActuatorType::RobStride00 => (
                                    Box::new(RobStride00::new(id, host_id, transport_tx.clone())),
                                    config.clone(),
                                ),
                                ActuatorType::RobStride01 => (
                                    Box::new(RobStride01::new(id, host_id, transport_tx.clone())),
                                    config.clone(),
                                ),
                                ActuatorType::RobStride02 => (
                                    Box::new(RobStride02::new(id, host_id, transport_tx.clone())),
                                    config.clone(),
                                ),
                                ActuatorType::RobStride03 => (
                                    Box::new(RobStride03::new(id, host_id, transport_tx.clone())),
                                    config.clone(),
                                ),
                                ActuatorType::RobStride04 => (
                                    Box::new(RobStride04::new(id, host_id, transport_tx.clone())),
                                    config.clone(),
                                ),
                            },
                            None => (
                                Box::new(RobStride04::new(id, host_id, transport_tx.clone())),
                                ActuatorConfiguration {
                                    actuator_type: ActuatorType::RobStride04,
                                    max_angle_change: Some(1.0),
                                    max_velocity: None,
                                    command_rate_hz: Some(100.0),
                                },
                            ),
                        };

                    let actuator_type = actuator.actuator_type();
                    actuators.insert(
                        id,
                        ActuatorRecord {
                            actuator,
                            state: ActuatorState {
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
                            },
                        },
                    );
                    debug!(
                        "Added actuator with ID: {} (type: {:?}) on {}",
                        id, actuator_type, transport_name
                    );
                }
            }

            time::sleep(Duration::from_millis(1)).await;
        }

        let discovered_ids = self.discovered_ids.read().await;
        Ok(discovered_ids.clone())
    }

    pub async fn run(&mut self, interval: Duration) -> Result<()> {
        info!("Starting supervisor");
        let mut interval = time::interval(interval);

        loop {
            interval.tick().await;

            {
                let mut actuators_snapshot = self.actuators.write().await;
                let num_actuators = actuators_snapshot.len();

                // Process actuators
                for (&id, record) in actuators_snapshot.iter_mut() {
                    if record.state.enabled {
                        if record.state.ready {
                            let feedback = match record.state.feedback.as_ref() {
                                Some(f) => f,
                                None => {
                                    warn!("No feedback available for actuator {}, skipping control {:?}", id, record.state.control_command);
                                    continue;
                                }
                            };
                            let now = SystemTime::now();
                            let mut command_valid = true;

                            // Check command rate limit if configured
                            if let Some(rate_hz) = record.state.configuration.command_rate_hz {
                                let min_interval = Duration::from_secs_f32(1.0 / rate_hz);
                                if let Ok(elapsed) = record.state.last_command.elapsed() {
                                    if elapsed < min_interval {
                                        trace!(
                                            "Skipping command for actuator {} due to rate limit ({:.1} Hz): {:.1}ms < {:.1}ms",
                                            id,
                                            rate_hz,
                                            elapsed.as_secs_f32() * 1000.0,
                                            min_interval.as_secs_f32() * 1000.0
                                        );
                                        command_valid = false;
                                    }
                                }
                            }

                            // Check angle change limit if configured
                            if let Some(max_angle_change) =
                                record.state.configuration.max_angle_change
                            {
                                let max_angle_change_percent = normalize_value(
                                    max_angle_change,
                                    -4.0 * std::f32::consts::PI,
                                    4.0 * std::f32::consts::PI,
                                    -100.0,
                                    100.0,
                                );

                                let angle_diff = (record.state.control_command.target_angle
                                    - feedback.angle)
                                    .abs();
                                if angle_diff > max_angle_change_percent {
                                    error!(
                                        "Actuator {} angle change too large: {:.3}% > {:.3}%, target={:.3}%, feedback={:.3}%",
                                        id, angle_diff, max_angle_change_percent, record.state.control_command.target_angle, feedback.angle
                                    );
                                    command_valid = false;
                                }
                            }

                            // Check velocity limit if configured
                            if let Some(max_velocity) = record.state.configuration.max_velocity {
                                if record.state.control_command.target_velocity.abs() > max_velocity
                                {
                                    error!(
                                        "Actuator {} velocity too large: {:.3} > {:.3}, target={:.3}, feedback={:.3}",
                                        id,
                                        record.state.control_command.target_velocity.abs(),
                                        max_velocity,
                                        record.state.control_command.target_velocity,
                                        feedback.velocity
                                    );
                                    command_valid = false;
                                }
                            }

                            if cfg!(feature = "instant_command") {
                                command_valid = false;
                            }

                            if command_valid {
                                if let Err(e) = record
                                    .actuator
                                    .control(record.state.control_command.clone())
                                    .await
                                {
                                    error!("Failed to control actuator {}: {}", id, e);
                                } else {
                                    record.state.last_command = now;
                                }
                            } else {
                                if let Err(e) = record.actuator.get_feedback().await {
                                    error!("Failed to get feedback from actuator {}: {}", id, e);
                                }
                            }
                        } else {
                            warn!(
                                "Actuator {} is not ready, {:?}",
                                id, record.state.control_command
                            );
                        }
                    } else {
                        if let Err(e) = record.actuator.get_feedback().await {
                            error!("Failed to get feedback from actuator {}: {}", id, e);
                        }
                    }
                }

                // Check timeouts and print stats with write lock
                drop(actuators_snapshot); // Drop read lock before taking write lock
                let mut actuators = self.actuators.write().await;

                for (&id, record) in actuators.iter_mut() {
                    if record.state.last_feedback.elapsed()? > self.feedback_timeout {
                        error!("Feedback timeout for actuator {}", id);
                        record.state.enabled = false;

                        if let Err(e) = record.actuator.disable(false).await {
                            error!("Failed to disable actuator {} after timeout: {}", id, e);
                        }
                    }
                }

                if self.last_stats_time.elapsed()? > Duration::from_secs(5) {
                    let total_messages: u64 = actuators
                        .values()
                        .map(|record| record.state.messages_received)
                        .sum();
                    info!(
                        "Messages received: {} (avg {:.1} Hz), len={}",
                        total_messages,
                        total_messages as f32 / (5.0 * num_actuators as f32),
                        num_actuators
                    );

                    for record in actuators.values_mut() {
                        record.state.messages_received = 0;
                    }
                    self.last_stats_time = SystemTime::now();
                }
            }
        }
    }

    pub async fn enable(&mut self, id: u8) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.enable().await?;
        record.state.enabled = true;
        Ok(())
    }

    pub async fn disable(&mut self, id: u8, clear_fault: bool) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.disable(clear_fault).await?;
        record.state.enabled = false;
        Ok(())
    }

    pub async fn configure(&mut self, id: u8, config: ControlConfig) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;

        let cmd = match record.state.actuator_type {
            ActuatorType::RobStride00 => RobStride00Command {
                kp: config.kp,
                kd: config.kd,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride01 => RobStride01Command {
                kp: config.kp,
                kd: config.kd,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride02 => RobStride02Command {
                kp: config.kp,
                kd: config.kd,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride03 => RobStride03Command {
                kp: config.kp,
                kd: config.kd,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride04 => RobStride04Command {
                kp: config.kp,
                kd: config.kd,
                ..Default::default()
            }
            .to_control_command(),
        };

        record.state.control_config = config.clone();
        record.state.control_command.kp = cmd.kp;
        record.state.control_command.kd = cmd.kd;

        // Set limits if provided
        if let Some(max_torque) = config.max_torque {
            record.actuator.set_max_torque(max_torque).await?;
        }
        if let Some(max_velocity) = config.max_velocity {
            record.actuator.set_max_velocity(max_velocity).await?;
        }
        if let Some(max_current) = config.max_current {
            record.actuator.set_max_current(max_current).await?;
        }

        Ok(())
    }

    pub async fn request_feedback(&self, id: u8) -> Result<()> {
        let actuators = self.actuators.read().await;
        let record = actuators.get(&id);
        if let Some(record) = record {
            if let Err(e) = record.actuator.get_feedback().await {
                error!("Failed to request feedback from actuator {}: {}", id, e);
            }
            trace!(event = "FeedbackRequested", motor_id = id, "Supervisor requested feedback from actuator");
        } else {
            error!("Actuator {} not found", id);
        }
        Ok(())
    }

    pub async fn command(
        &mut self,
        id: u8,
        position: f32,
        velocity: f32,
        torque: f32,
    ) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;

        let position = denormalize_radians(position, record.state.half_revolutions);

        let cmd = match record.state.actuator_type {
            ActuatorType::RobStride00 => RobStride00Command {
                target_angle_rad: position,
                target_velocity_rads: velocity,
                torque_nm: torque,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride01 => RobStride01Command {
                target_angle_rad: position,
                target_velocity_rads: velocity,
                torque_nm: torque,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride02 => RobStride02Command {
                target_angle_rad: position,
                target_velocity_rads: velocity,
                torque_nm: torque,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride03 => RobStride03Command {
                target_angle_rad: position,
                target_velocity_rads: velocity,
                torque_nm: torque,
                ..Default::default()
            }
            .to_control_command(),
            ActuatorType::RobStride04 => RobStride04Command {
                target_angle_rad: position,
                target_velocity_rads: velocity,
                torque_nm: torque,
                ..Default::default()
            }
            .to_control_command(),
        };

        record.state.control_command.target_angle = cmd.target_angle;
        record.state.control_command.target_velocity = cmd.target_velocity;
        record.state.control_command.torque = cmd.torque;
        trace!(event = "MITCommandSent", motor_id = id, cmd = ?cmd, "Supervisor sending command to actuator");
        if cfg!(feature = "instant_command") {
            record
                .actuator
                .control(record.state.control_command.clone())
                .await?;
        }

        Ok(())
    }

    pub async fn set_id(&mut self, id: u8, new_id: u8) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.set_id(new_id).await?;

        let record = actuators.remove(&id).unwrap();
        actuators.insert(new_id, record);
        Ok(())
    }

    pub async fn get_uuid(&mut self, id: u8) -> Result<()> {
        let actuators = self.actuators.read().await;
        let record = actuators
            .get(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.get_uuid().await
    }

    pub async fn control(&mut self, id: u8, cmd: ControlCommand) -> Result<()> {
        let actuators = self.actuators.read().await;
        let record = actuators
            .get(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.control(cmd).await
    }

    pub async fn get_feedback(&self, id: u8) -> Result<Option<(FeedbackFrame, SystemTime)>> {
        let actuators = self.actuators.read().await;
        let record = actuators.get(&id);
        if let Some(record) = record {
            if let Some(mut feedback) = record.state.feedback.clone() {
                let typed_feedback: Box<dyn TypedFeedbackData> = match record.state.actuator_type {
                    ActuatorType::RobStride00 => {
                        Box::new(RobStride00Feedback::from_feedback_frame(feedback.clone()))
                    }
                    ActuatorType::RobStride01 => {
                        Box::new(RobStride01Feedback::from_feedback_frame(feedback.clone()))
                    }
                    ActuatorType::RobStride02 => {
                        Box::new(RobStride02Feedback::from_feedback_frame(feedback.clone()))
                    }
                    ActuatorType::RobStride03 => {
                        Box::new(RobStride03Feedback::from_feedback_frame(feedback.clone()))
                    }
                    ActuatorType::RobStride04 => {
                        Box::new(RobStride04Feedback::from_feedback_frame(feedback.clone()))
                    }
                };

                feedback.angle = typed_feedback.angle_rad();
                feedback.velocity = typed_feedback.velocity_rads();
                feedback.torque = typed_feedback.torque_nm();

                // Log feedback information
                debug!("Motor {} feedback:", id);
                debug!("  Angle: {:?}", feedback.angle);
                debug!("  Velocity: {:?}", feedback.velocity);
                debug!("  Torque: {:?}", feedback.torque);
                debug!("  Temperature: {:?}", feedback.temperature);
                debug!("  Faults:");
                debug!("    Uncalibrated: {:?}", feedback.fault_uncalibrated);
                debug!("    Hall encoding: {:?}", feedback.fault_hall_encoding);
                debug!(
                    "    Magnetic encoding: {:?}",
                    feedback.fault_magnetic_encoding
                );
                debug!(
                    "    Over temperature: {:?}",
                    feedback.fault_over_temperature
                );
                debug!("    Overcurrent: {:?}", feedback.fault_overcurrent);
                debug!("    Undervoltage: {:?}", feedback.fault_undervoltage);

                feedback.angle = normalize_radians(feedback.angle).0;
                return Ok(Some((feedback, record.state.last_feedback)));
            }
        }
        Ok(None)
    }

    pub async fn zero(&mut self, id: u8) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let record = actuators
            .get_mut(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.set_zero().await
    }

    pub async fn change_id(&mut self, id: u8, new_id: u8) -> Result<()> {
        let mut actuators = self.actuators.write().await;
        let mut record = actuators
            .remove(&id)
            .ok_or_else(|| eyre::eyre!("Actuator not found"))?;
        record.actuator.set_id(new_id).await?;
        actuators.insert(new_id, record);
        Ok(())
    }
}

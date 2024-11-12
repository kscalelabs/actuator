use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration;

use crate::motor::{MotorControlParams, MotorFeedback, MotorSdoParams, Motors};
use crate::types::{MotorType, RunMode};
use log::{error, info};

pub struct MotorsSupervisor {
    motors: Arc<Mutex<Motors>>,
    target_params: Arc<RwLock<HashMap<u8, MotorControlParams>>>,
    running: Arc<RwLock<bool>>,
    latest_feedback: Arc<RwLock<HashMap<u8, MotorFeedback>>>,
    motors_to_zero: Arc<Mutex<HashSet<u8>>>,
    motors_to_set_sdo: Arc<Mutex<HashMap<u8, MotorSdoParams>>>,
    paused: Arc<RwLock<bool>>,
    restart: Arc<Mutex<bool>>,
    total_commands: Arc<RwLock<u64>>,
    failed_commands: Arc<RwLock<HashMap<u8, u64>>>,
    max_update_rate: Arc<RwLock<f64>>,
    actual_update_rate: Arc<RwLock<f64>>,
    serial: Arc<RwLock<bool>>,
}

impl MotorsSupervisor {
    pub fn new(
        port_name: &str,
        motor_infos: &HashMap<u8, MotorType>,
        verbose: bool,
        max_update_rate: f64,
        zero_on_init: bool,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        // Initialize Motors
        let motors = Motors::new(port_name, motor_infos, verbose)?;

        // Get default KP/KD values for all motors.
        let target_params = motors
            .motor_configs
            .keys()
            .map(|id| {
                (
                    *id,
                    MotorControlParams {
                        position: 0.0,
                        velocity: 0.0,
                        kp: 0.0,
                        kd: 0.0,
                        torque: 0.0,
                    },
                )
            })
            .collect::<HashMap<u8, MotorControlParams>>();

        // Find motors that need to be zeroed on initialization.
        let zero_on_init_motors = motors
            .motor_configs
            .iter()
            .filter(|(_, &config)| config.zero_on_init || zero_on_init)
            .map(|(&id, _)| id)
            .collect::<HashSet<u8>>();

        let motor_ids: Vec<u8> = motor_infos.keys().cloned().collect();
        let total_commands = 0;
        let failed_commands = motor_ids.iter().map(|&id| (id, 0)).collect();

        let controller = MotorsSupervisor {
            motors: Arc::new(Mutex::new(motors)),
            target_params: Arc::new(RwLock::new(target_params)),
            running: Arc::new(RwLock::new(true)),
            latest_feedback: Arc::new(RwLock::new(HashMap::new())),
            motors_to_zero: Arc::new(Mutex::new(zero_on_init_motors)),
            motors_to_set_sdo: Arc::new(Mutex::new(HashMap::new())),
            paused: Arc::new(RwLock::new(false)),
            restart: Arc::new(Mutex::new(false)),
            total_commands: Arc::new(RwLock::new(total_commands)),
            failed_commands: Arc::new(RwLock::new(failed_commands)),
            max_update_rate: Arc::new(RwLock::new(max_update_rate)),
            actual_update_rate: Arc::new(RwLock::new(0.0)),
            serial: Arc::new(RwLock::new(true)),
        };

        controller.start_control_thread();

        Ok(controller)
    }

    fn start_control_thread(&self) {
        let motors = Arc::clone(&self.motors);
        let target_params = Arc::clone(&self.target_params);
        let running = Arc::clone(&self.running);
        let latest_feedback = Arc::clone(&self.latest_feedback);
        let motors_to_zero = Arc::clone(&self.motors_to_zero);
        let paused = Arc::clone(&self.paused);
        let restart = Arc::clone(&self.restart);
        let motors_to_set_sdo = Arc::clone(&self.motors_to_set_sdo);
        let total_commands = Arc::clone(&self.total_commands);
        let failed_commands = Arc::clone(&self.failed_commands);
        let max_update_rate = Arc::clone(&self.max_update_rate);
        let actual_update_rate = Arc::clone(&self.actual_update_rate);
        let serial = Arc::clone(&self.serial);

        thread::spawn(move || {
            let mut motors = motors.lock().unwrap();

            // Runs pre-flight checks.
            if let Err(err) = motors.send_resets() {
                error!("Failed to send resets: {}", err);
                *running.write().unwrap() = false;
                return;
            }
            if let Err(err) = motors.send_starts() {
                error!("Failed to send starts: {}", err);
                *running.write().unwrap() = false;
                return;
            }

            info!("Pre-flight checks completed successfully");
            let _ = motors.send_set_mode(RunMode::MitMode);

            let mut last_update_time = std::time::Instant::now();

            loop {
                {
                    // If not running, break the loop.
                    if !*running.read().unwrap() {
                        break;
                    }
                }

                {
                    // If paused, just wait a short time without sending any commands.
                    if *paused.read().unwrap() {
                        thread::sleep(Duration::from_millis(10));
                        continue;
                    }
                }

                {
                    // If restart is requested, reset and restart the motors.
                    let mut restart = restart.lock().unwrap();
                    if *restart {
                        *restart = false;
                        let _ = motors.send_resets();
                        let _ = motors.send_starts();
                    }
                }

                let loop_start_time = std::time::Instant::now();

                {
                    // Send zero torque commands to motors that need to be zeroed.
                    let mut motor_ids_to_zero = motors_to_zero.lock().unwrap();
                    if !motor_ids_to_zero.is_empty() {
                        let motor_ids = motor_ids_to_zero.iter().cloned().collect::<Vec<u8>>();
                        let _ = motors.zero_motors(&motor_ids);
                        motor_ids_to_zero.clear();
                    }
                }

                {
                    // Send updated sdo parameters to motors that need them.
                    let mut motors_to_set_sdo = motors_to_set_sdo.lock().unwrap();
                    if !motors_to_set_sdo.is_empty() {
                        for (motor_id, params) in motors_to_set_sdo.iter_mut() {
                            if let Some(torque_limit) = params.torque_limit {
                                motors.set_torque_limit(*motor_id, torque_limit).unwrap();
                            }
                            if let Some(speed_limit) = params.speed_limit {
                                motors.set_speed_limit(*motor_id, speed_limit).unwrap();
                            }
                            if let Some(current_limit) = params.current_limit {
                                motors.set_current_limit(*motor_id, current_limit).unwrap();
                            }
                        }
                        motors_to_set_sdo.clear();
                    }
                }

                {
                    let params_copy = {
                        let target_params = target_params.read().unwrap();
                        target_params.clone()
                    };

                    if !params_copy.is_empty() {
                        match motors.send_motor_controls(&params_copy, *serial.read().unwrap()) {
                            Ok(feedbacks) => {
                                let mut latest_feedback = latest_feedback.write().unwrap();
                                let mut failed_commands = failed_commands.write().unwrap();
                                for &motor_id in params_copy.keys() {
                                    if let Some(feedback) = feedbacks.get(&motor_id) {
                                        latest_feedback.insert(motor_id, feedback.clone());
                                    } else {
                                        failed_commands.entry(motor_id).and_modify(|e| *e += 1);
                                    }
                                }
                                *total_commands.write().unwrap() += 1;
                            }
                            Err(_) => {}
                        }
                    }
                }

                {
                    // Calculate actual update rate, as an exponentially weighted moving average.
                    let elapsed = loop_start_time.duration_since(last_update_time);
                    last_update_time = loop_start_time;
                    let current_rate = 1.0 / elapsed.as_secs_f64();
                    let prev_actual_update_rate = *actual_update_rate.read().unwrap();
                    *actual_update_rate.write().unwrap() =
                        prev_actual_update_rate * 0.9 + current_rate * 0.1;
                }

                {
                    // Sleep to maintain maximum update rate.
                    let target_duration =
                        Duration::from_secs_f64(1.0 / *max_update_rate.read().unwrap());
                    let elapsed = loop_start_time.elapsed();
                    let min_sleep_duration = Duration::from_micros(1);
                    if target_duration > elapsed + min_sleep_duration {
                        thread::sleep(target_duration - elapsed);
                    } else {
                        thread::sleep(min_sleep_duration);
                    }
                }
            }

            let motor_ids: Vec<u8> = motors.motor_configs.keys().cloned().collect::<Vec<u8>>();
            let zero_torque_sets: HashMap<u8, MotorControlParams> = HashMap::from_iter(
                motor_ids
                    .iter()
                    .map(|id| (*id, MotorControlParams::default())),
            );
            let _ = motors.send_motor_controls(&zero_torque_sets, true);
            let _ = motors.send_resets();
        });
    }

    // Updated methods to access the command counters
    pub fn get_total_commands(&self) -> u64 {
        *self.total_commands.read().unwrap()
    }

    pub fn get_failed_commands(&self, motor_id: u8) -> Result<u64, std::io::Error> {
        self.failed_commands
            .read()
            .unwrap()
            .get(&motor_id)
            .copied()
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn reset_command_counters(&self) {
        *self.total_commands.write().unwrap() = 0;
        *self.failed_commands.write().unwrap() = HashMap::new();
    }

    pub fn set_all_params(&self, params: HashMap<u8, MotorControlParams>) {
        let mut target_params = self.target_params.write().unwrap();
        *target_params = params;
    }

    pub fn set_params(
        &self,
        motor_id: u8,
        params: MotorControlParams,
    ) -> Result<(), std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        target_params.insert(motor_id, params);
        Ok(())
    }

    pub fn set_positions(&self, positions: HashMap<u8, f32>) -> Result<(), std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        for (motor_id, position) in positions {
            target_params.get_mut(&motor_id).unwrap().position = position;
        }
        Ok(())
    }

    pub fn set_position(&self, motor_id: u8, position: f32) -> Result<f32, std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.position = position;
            Ok(params.position)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Motor ID {} not found", motor_id),
            ))
        }
    }

    pub fn get_position(&self, motor_id: u8) -> Result<f32, std::io::Error> {
        let target_params = self.target_params.read().unwrap();
        target_params
            .get(&motor_id)
            .map(|params| params.position)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn set_velocities(&self, velocities: HashMap<u8, f32>) -> Result<(), std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        for (motor_id, velocity) in velocities {
            target_params.get_mut(&motor_id).unwrap().velocity = velocity;
        }
        Ok(())
    }

    pub fn set_velocity(&self, motor_id: u8, velocity: f32) -> Result<f32, std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.velocity = velocity;
            Ok(params.velocity)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Motor ID {} not found", motor_id),
            ))
        }
    }

    pub fn get_velocity(&self, motor_id: u8) -> Result<f32, std::io::Error> {
        let target_params = self.target_params.read().unwrap();
        target_params
            .get(&motor_id)
            .map(|params| params.velocity)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn set_kp(&self, motor_id: u8, kp: f32) -> Result<f32, std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.kp = kp.max(0.0); // Clamp kp to be non-negative.
            Ok(params.kp)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Motor ID {} not found", motor_id),
            ))
        }
    }

    pub fn get_kp(&self, motor_id: u8) -> Result<f32, std::io::Error> {
        let target_params = self.target_params.read().unwrap();
        target_params
            .get(&motor_id)
            .map(|params| params.kp)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn set_kd(&self, motor_id: u8, kd: f32) -> Result<f32, std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.kd = kd.max(0.0); // Clamp kd to be non-negative.
            Ok(params.kd)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Motor ID {} not found", motor_id),
            ))
        }
    }

    pub fn get_kd(&self, motor_id: u8) -> Result<f32, std::io::Error> {
        let target_params = self.target_params.read().unwrap();
        target_params
            .get(&motor_id)
            .map(|params| params.kd)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn set_torque_limit(&self, motor_id: u8, torque_limit: f32) -> Result<f32, std::io::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock().unwrap();
        motors_to_set_sdo.insert(motor_id, MotorSdoParams { torque_limit: Some(torque_limit), speed_limit: None, current_limit: None });
        Ok(torque_limit)
    }

    pub fn set_speed_limit(&self, motor_id: u8, speed_limit: f32) -> Result<f32, std::io::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock().unwrap();
        motors_to_set_sdo.insert(motor_id, MotorSdoParams { torque_limit: None, speed_limit: Some(speed_limit), current_limit: None });
        Ok(speed_limit)
    }

    pub fn set_current_limit(&self, motor_id: u8, current_limit: f32) -> Result<f32, std::io::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock().unwrap();
        motors_to_set_sdo.insert(motor_id, MotorSdoParams { torque_limit: None, speed_limit: None, current_limit: Some(current_limit) });
        Ok(current_limit)
    }

    pub fn set_torque(&self, motor_id: u8, torque: f32) -> Result<f32, std::io::Error> {
        let mut target_params = self.target_params.write().unwrap();
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.torque = torque;
            Ok(params.torque)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                format!("Motor ID {} not found", motor_id),
            ))
        }
    }

    pub fn get_torque(&self, motor_id: u8) -> Result<f32, std::io::Error> {
        let target_params = self.target_params.read().unwrap();
        target_params
            .get(&motor_id)
            .map(|params| params.torque)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::NotFound,
                    format!("Motor ID {} not found", motor_id),
                )
            })
    }

    pub fn add_motor_to_zero(&self, motor_id: u8) -> Result<(), std::io::Error> {
        // We need to set the motor parameters to zero to avoid the motor
        // rapidly changing to the new target after it is zeroed.
        self.set_torque(motor_id, 0.0)?;
        self.set_position(motor_id, 0.0)?;
        self.set_velocity(motor_id, 0.0)?;
        let mut motors_to_zero = self.motors_to_zero.lock().unwrap();
        motors_to_zero.insert(motor_id);
        Ok(())
    }

    pub fn get_latest_feedback(&self) -> HashMap<u8, MotorFeedback> {
        let latest_feedback = self.latest_feedback.read().unwrap();
        latest_feedback.clone()
    }

    pub fn toggle_pause(&self) {
        let mut paused = self.paused.write().unwrap();
        *paused = !*paused;
    }

    pub fn reset(&self) {
        let mut restart = self.restart.lock().unwrap();
        *restart = true;
    }

    pub fn stop(&self) {
        {
            let mut running = self.running.write().unwrap();
            *running = false;
        }
        thread::sleep(Duration::from_millis(200));
    }

    pub fn is_running(&self) -> bool {
        *self.running.read().unwrap()
    }

    pub fn set_max_update_rate(&self, rate: f64) {
        let mut max_rate = self.max_update_rate.write().unwrap();
        *max_rate = rate;
    }

    pub fn get_actual_update_rate(&self) -> f64 {
        *self.actual_update_rate.read().unwrap()
    }

    pub fn get_serial(&self) -> bool {
        *self.serial.read().unwrap()
    }

    pub fn toggle_serial(&self) -> bool {
        let mut serial = self.serial.write().unwrap();
        *serial = !*serial;
        *serial
    }
}

impl Drop for MotorsSupervisor {
    fn drop(&mut self) {
        self.stop();
    }
}

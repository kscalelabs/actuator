use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration;

use crate::motor::{MotorControlParams, MotorFeedback, MotorSdoParams, Motors};
use crate::types::{MotorType, RunMode};
use log::{error, info};
use eyre::{eyre, Result};

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
            let motors_guard = match motors.lock() {
                Ok(guard) => guard,
                Err(err) => {
                    error!("Failed to lock motors: {}", err);
                    return;
                }
            };
            let mut motors = motors_guard;

            // Runs pre-flight checks.
            if let Err(err) = motors.send_resets() {
                error!("Failed to send resets: {}", err);
                if let Ok(mut running_guard) = running.write() {
                    *running_guard = false;
                } else {
                    error!("Failed to acquire write lock on running flag");
                }
                return;
            }
            if let Err(err) = motors.send_starts() {
                error!("Failed to send starts: {}", err);
                if let Ok(mut running_guard) = running.write() {
                    *running_guard = false;
                } else {
                    error!("Failed to acquire write lock on running flag");
                }
                return;
            }

            // Add another pre-flight check to see wait for feedback from each motor and check that they are all valid (MotorMode::Motor).
            // Additionally, get the current position of each motor and check that it is not zero (should always be positive).
            // Save the position and set the target_ for each motor to this position.
            // The below is cursor generated code and definitely doesn't work.
            let mut all_motors_valid = true;
            let mut motor_positions = HashMap::<u8, f32>::new();
            for motor_id in motors.motor_configs.keys() {
                let feedback = motors.get_feedback(*motor_id).unwrap();
                if feedback.mode != MotorMode::Motor {
                    all_motors_valid = false;
                }
                motor_positions.insert(*motor_id, feedback.p);
            }

            info!("Pre-flight checks completed successfully");
            let _ = motors.send_set_mode(RunMode::MitMode);

            let mut last_update_time = std::time::Instant::now();

            loop {
                {
                    // If not running, break the loop.
                    if let Ok(running_guard) = running.read() {
                        if !*running_guard {
                            break;
                        }
                    } else {
                        error!("Failed to acquire read lock on running flag");
                    }
                }

                {
                    // If paused, just wait a short time without sending any commands.
                    if let Ok(paused_guard) = paused.read() {
                        if *paused_guard {
                            thread::sleep(Duration::from_millis(10));
                            continue;
                        }
                    } else {
                        error!("Failed to acquire read lock on paused flag");
                    }
                }

                {
                    // If restart is requested, reset and restart the motors.
                    if let Ok(mut restart_guard) = restart.lock() {
                        if *restart_guard {
                            *restart_guard = false;
                            let _ = motors.send_resets();
                            let _ = motors.send_starts();
                        }
                    } else {
                        error!("Failed to acquire lock on restart flag");
                    }
                }

                let loop_start_time = std::time::Instant::now();

                {
                    // Send zero torque commands to motors that need to be zeroed.
                    if let Ok(mut motor_ids_to_zero) = motors_to_zero.lock() {
                        if !motor_ids_to_zero.is_empty() {
                            let motor_ids = motor_ids_to_zero.iter().cloned().collect::<Vec<u8>>();
                            let _ = motors.zero_motors(&motor_ids);
                            motor_ids_to_zero.clear();
                        }
                    } else {
                        error!("Failed to acquire lock on motors_to_zero");
                    }
                }

                {
                    // Send updated sdo parameters to motors that need them.
                    if let Ok(mut motors_to_set_sdo) = motors_to_set_sdo.lock() {
                        if !motors_to_set_sdo.is_empty() {
                            for (motor_id, params) in motors_to_set_sdo.iter_mut() {
                                if let Some(torque_limit) = params.torque_limit {
                                    if let Err(e) = motors.set_torque_limit(*motor_id, torque_limit) {
                                        error!("Failed to set torque limit for motor {}: {}", motor_id, e);
                                    }
                                }
                                if let Some(speed_limit) = params.speed_limit {
                                    if let Err(e) = motors.set_speed_limit(*motor_id, speed_limit) {
                                        error!("Failed to set speed limit for motor {}: {}", motor_id, e);
                                    }
                                }
                                if let Some(current_limit) = params.current_limit {
                                    if let Err(e) = motors.set_current_limit(*motor_id, current_limit) {
                                        error!("Failed to set current limit for motor {}: {}", motor_id, e);
                                    }
                                }
                            }
                            motors_to_set_sdo.clear();
                        }
                    } else {
                        error!("Failed to acquire lock on motors_to_set_sdo");
                    }
                }

                {
                    let params_copy = {
                        if let Ok(target_params) = target_params.read() {
                            target_params.clone()
                        } else {
                            error!("Failed to acquire read lock on target_params");
                            HashMap::new()
                        }
                    };

                    if !params_copy.is_empty() {
                        let serial_value = serial.read().map_or_else(
                            |e| {
                                error!("Failed to acquire read lock on serial: {}", e);
                                true  // Default to true if we can't read the lock
                            },
                            |val| *val
                        );

                        match motors.send_motor_controls(&params_copy, serial_value) {
                            Ok(feedbacks) => {
                                let mut latest_feedback = match latest_feedback.write() {
                                    Ok(guard) => guard,
                                    Err(e) => {
                                        error!("Failed to acquire write lock on latest_feedback: {}", e);
                                        continue;
                                    }
                                };
                                let mut failed_commands = match failed_commands.write() {
                                    Ok(guard) => guard,
                                    Err(e) => {
                                        error!("Failed to acquire write lock on failed_commands: {}", e);
                                        continue;
                                    }
                                };
                                for &motor_id in params_copy.keys() {
                                    if let Some(feedback) = feedbacks.get(&motor_id) {
                                        latest_feedback.insert(motor_id, feedback.clone());
                                    } else {
                                        failed_commands.entry(motor_id).and_modify(|e| *e += 1);
                                    }
                                }
                                if let Ok(mut total) = total_commands.write() {
                                    *total += 1;
                                } else {
                                    error!("Failed to acquire write lock on total_commands");
                                }
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

                    let prev_actual_update_rate = actual_update_rate.read().map_or_else(
                        |e| {
                            error!("Failed to acquire read lock on actual_update_rate: {}", e);
                            0.0
                        },
                        |rate| *rate
                    );

                    if let Ok(mut actual) = actual_update_rate.write() {
                        *actual = prev_actual_update_rate * 0.9 + current_rate * 0.1;
                    } else {
                        error!("Failed to acquire write lock on actual_update_rate");
                    }
                }

                {
                    // Sleep to maintain maximum update rate.
                    let target_duration =
                        Duration::from_secs_f64(1.0 / max_update_rate.read().map_or_else(
                            |e| {
                                error!("Failed to acquire read lock on max_update_rate: {}", e);
                                0.0
                            },
                            |rate| *rate
                        ));
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
    pub fn get_total_commands(&self) -> Result<u64, eyre::Error> {
        Ok(*self.total_commands.read().map_err(|e| eyre!(
            "Failed to acquire read lock on total_commands: {}", e
        ))?)
    }
    pub fn get_failed_commands(&self, motor_id: u8) -> Result<u64, eyre::Error> {
        self.failed_commands
            .read()
            .map_err(|e| eyre!(
                "Failed to acquire read lock: {}", e
            ))?
            .get(&motor_id)
            .copied()
            .ok_or_else(|| {
                eyre!(
                    "Motor ID {} not found", motor_id
                )
            })
    }

    pub fn reset_command_counters(&self) -> Result<(), eyre::Error> {
        self.total_commands
            .write()
            .map_err(|e| eyre!(
                "Failed to acquire write lock on total_commands: {}", e
            ))
            .map(|mut total| *total = 0)?;

        self.failed_commands
            .write()
            .map_err(|e| eyre!(
                "Failed to acquire write lock on failed_commands: {}", e
            ))
            .map(|mut failed| *failed = HashMap::new())?;

        Ok(())
    }

    pub fn set_all_params(&self, params: HashMap<u8, MotorControlParams>) -> Result<(), eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!(
            format!("Failed to acquire write lock on target_params: {}", e)
        ))?;
        *target_params = params;
        Ok(())
    }

    pub fn set_params(
        &self,
        motor_id: u8,
        params: MotorControlParams,
    ) -> Result<(), eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!(
            format!("Failed to acquire write lock on target_params: {}", e)
        ))?;
        target_params.insert(motor_id, params);
        Ok(())
    }

    pub fn set_positions(&self, positions: HashMap<u8, f32>) -> Result<(), eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!(
            "Failed to acquire write lock on target_params: {}", e
        ))?;
        for (motor_id, position) in positions {
            if let Some(params) = target_params.get_mut(&motor_id) {
                params.position = position;
            } else {
                return Err(eyre!("Motor ID {} not found", motor_id));
            }
        }
        Ok(())
    }

    pub fn set_position(&self, motor_id: u8, position: f32) -> Result<f32, eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!(
            "Failed to acquire write lock on target_params: {}", e
        ))?;
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.position = position;
            Ok(params.position)
        } else {
            Err(eyre!(
                "Motor ID {} not found", motor_id
            ))
        }
    }

    pub fn get_position(&self, motor_id: u8) -> Result<f32, eyre::Error> {
        let target_params = self.target_params.read()
            .map_err(|e| eyre!("Failed to acquire read lock on target_params: {}", e))?;
        target_params
            .get(&motor_id)
            .map(|params| params.position)
            .ok_or_else(|| eyre!("Motor ID {} not found", motor_id))
    }

    pub fn set_velocities(&self, velocities: HashMap<u8, f32>) -> Result<(), eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!("Failed to acquire write lock on target_params: {}", e))?;
        for (motor_id, velocity) in velocities {
            if let Some(params) = target_params.get_mut(&motor_id) {
                params.velocity = velocity;
            } else {
                return Err(eyre!("Motor ID {} not found", motor_id));
            }
        }
        Ok(())
    }

    pub fn set_velocity(&self, motor_id: u8, velocity: f32) -> Result<f32, eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!("Failed to acquire write lock on target_params: {}", e))?;
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.velocity = velocity;
            Ok(params.velocity)
        } else {
            Err(eyre!("Motor ID {} not found", motor_id))
        }
    }

    pub fn get_velocity(&self, motor_id: u8) -> Result<f32, eyre::Error> {
        let target_params = self.target_params.read()
            .map_err(|e| eyre!("Failed to acquire read lock on target_params: {}", e))?;
        target_params
            .get(&motor_id)
            .map(|params| params.velocity)
            .ok_or_else(|| eyre!("Motor ID {} not found", motor_id))
    }

    pub fn set_kp(&self, motor_id: u8, kp: f32) -> Result<f32, eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!("Failed to acquire write lock on target_params: {}", e))?;
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.kp = kp.max(0.0); // Clamp kp to be non-negative.
            Ok(params.kp)
        } else {
            Err(eyre!("Motor ID {} not found", motor_id))
        }
    }

    pub fn get_kp(&self, motor_id: u8) -> Result<f32, eyre::Error> {
        let target_params = self.target_params.read()
            .map_err(|e| eyre!("Failed to acquire read lock on target_params: {}", e))?;
        target_params
            .get(&motor_id)
            .map(|params| params.kp)
            .ok_or_else(|| eyre!("Motor ID {} not found", motor_id))
    }

    pub fn set_kd(&self, motor_id: u8, kd: f32) -> Result<f32, eyre::Error> {
        let mut target_params = self.target_params.write().map_err(|e| eyre!("Failed to acquire write lock on target_params: {}", e))?;
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.kd = kd.max(0.0); // Clamp kd to be non-negative.
            Ok(params.kd)
        } else {
            Err(eyre!("Motor ID {} not found", motor_id))
        }
    }

    pub fn get_kd(&self, motor_id: u8) -> Result<f32, eyre::Error> {
        let target_params = self.target_params.read()
            .map_err(|e| eyre!("Failed to read target params: {}", e))?;
        
        target_params
            .get(&motor_id)
            .map(|params| params.kd)
            .ok_or_else(|| eyre!("Motor ID {} not found", motor_id))
    }

    pub fn set_torque_limit(&self, motor_id: u8, torque_limit: f32) -> Result<f32, eyre::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock()
            .map_err(|e| eyre!("Failed to lock motors_to_set_sdo: {}", e))?;
        
        motors_to_set_sdo.insert(
            motor_id, 
            MotorSdoParams { 
                torque_limit: Some(torque_limit), 
                speed_limit: None, 
                current_limit: None 
            }
        );
        Ok(torque_limit)
    }

    pub fn set_speed_limit(&self, motor_id: u8, speed_limit: f32) -> Result<f32, eyre::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock()
            .map_err(|e| eyre!("Failed to lock motors_to_set_sdo: {}", e))?;
        
        motors_to_set_sdo.insert(
            motor_id, 
            MotorSdoParams { 
                torque_limit: None, 
                speed_limit: Some(speed_limit), 
                current_limit: None 
            }
        );
        Ok(speed_limit)
    }

    pub fn set_current_limit(&self, motor_id: u8, current_limit: f32) -> Result<f32, eyre::Error> {
        let mut motors_to_set_sdo = self.motors_to_set_sdo.lock()
            .map_err(|e| eyre!("Failed to lock motors_to_set_sdo: {}", e))?;
        
        motors_to_set_sdo.insert(
            motor_id, 
            MotorSdoParams { 
                torque_limit: None, 
                speed_limit: None, 
                current_limit: Some(current_limit) 
            }
        );
        Ok(current_limit)
    }

    pub fn set_torque(&self, motor_id: u8, torque: f32) -> Result<f32, eyre::Error> {
        let mut target_params = self.target_params.write()
            .map_err(|e| eyre!("Failed to acquire write lock on target_params: {}", e))?;
        if let Some(params) = target_params.get_mut(&motor_id) {
            params.torque = torque;
            Ok(params.torque)
        } else {
            Err(eyre!("Motor ID {} not found", motor_id))
        }
    }

    pub fn get_torque(&self, motor_id: u8) -> Result<f32, eyre::Error> {
        let target_params = self.target_params.read()
            .map_err(|e| eyre!("Failed to read target params: {}", e))?;
        target_params
            .get(&motor_id)
            .map(|params| params.torque)
            .ok_or_else(|| eyre!("Motor ID {} not found", motor_id))
    }

    pub fn add_motor_to_zero(&self, motor_id: u8) -> Result<(), eyre::Error> {
        // We need to set the motor parameters to zero to avoid the motor
        // rapidly changing to the new target after it is zeroed.
        self.set_torque(motor_id, 0.0)?;
        self.set_position(motor_id, 0.0)?;
        self.set_velocity(motor_id, 0.0)?;
        let mut motors_to_zero = self.motors_to_zero.lock()
            .map_err(|e| eyre!("Failed to lock motors_to_zero: {}", e))?;
        motors_to_zero.insert(motor_id);
        Ok(())
    }

    pub fn get_latest_feedback(&self) -> Result<HashMap<u8, MotorFeedback>, eyre::Error> {
        let latest_feedback = self.latest_feedback.read()
            .map_err(|e| eyre!("Failed to read latest_feedback: {}", e))?;
        Ok(latest_feedback.clone())
    }

    pub fn toggle_pause(&self) -> Result<(), eyre::Error> {
        let mut paused = self.paused.write().map_err(|e| eyre!("Failed to acquire write lock on paused: {}", e))?;
        *paused = !*paused;
        Ok(())
    }

    pub fn reset(&self) -> Result<(), eyre::Error> {
        let mut restart = self.restart.lock().map_err(|e| eyre!("Failed to lock restart: {}", e))?;
        *restart = true;
        Ok(())
    }

    pub fn stop(&self) -> Result<(), eyre::Error> {
        {
            let mut running = self.running.write().map_err(|e| eyre!("Failed to acquire write lock on running: {}", e))?;
            *running = false;
        }
        thread::sleep(Duration::from_millis(200));
        Ok(())
    }

    pub fn is_running(&self) -> Result<bool, eyre::Error> {
        Ok(*self.running.read().map_err(|e| eyre!("Failed to read running: {}", e))?)
    }

    pub fn set_max_update_rate(&self, rate: f64) -> Result<(), eyre::Error> {
        let mut max_rate = self.max_update_rate.write().map_err(|e| eyre!("Failed to acquire write lock on max_update_rate: {}", e))?;
        *max_rate = rate;
        Ok(())
    }

    pub fn get_actual_update_rate(&self) -> Result<f64, eyre::Error> {
        Ok(*self.actual_update_rate.read().map_err(|e| eyre!("Failed to read actual_update_rate: {}", e))?)
    }

    pub fn get_serial(&self) -> Result<bool, eyre::Error> {
        Ok(*self.serial.read().map_err(|e| eyre!("Failed to read serial: {}", e))?)
    }

    pub fn toggle_serial(&self) -> Result<bool, eyre::Error> {
        let mut serial = self.serial.write().map_err(|e| eyre!("Failed to acquire write lock on serial: {}", e))?;
        *serial = !*serial;
        Ok(*serial)
    }
}

impl Drop for MotorsSupervisor {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

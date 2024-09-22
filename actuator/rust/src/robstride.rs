use crate::ActuatorImpl;
use log::{debug, info};
use std::collections::HashMap;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use tokio::sync::Mutex as AsyncMutex;
use tokio::time::{sleep, Duration, Instant};

#[derive(Clone)]
struct PIDParams {
    pos_kp: f64,
    pos_kd: f64,
    vel_kp: f64,
    vel_kd: f64,
    tor_kp: f64,
    tor_kd: f64,
}

pub struct Robstride {
    debug_mode: bool,
    target_positions: Arc<Mutex<HashMap<u8, f64>>>,
    can_message_count: AtomicUsize,
    last_message_time: Arc<Mutex<Instant>>,
    pid_params: Mutex<HashMap<u8, PIDParams>>,
    send_lock: AsyncMutex<()>,
}

fn map_float_to_u8(value: f64, min: f64, max: f64) -> u8 {
    ((value.clamp(min, max) - min) / (max - min) * 255.0).round() as u8
}

impl Robstride {
    pub fn new(debug_mode: bool) -> Self {
        Robstride {
            debug_mode,
            target_positions: Arc::new(Mutex::new(HashMap::new())),
            can_message_count: AtomicUsize::new(0),
            last_message_time: Arc::new(Mutex::new(Instant::now())),
            pid_params: Mutex::new(HashMap::new()),
            send_lock: AsyncMutex::new(()),
        }
    }

    async fn send_can_message(&self, actuator_id: u8, message: [u8; 8]) {
        let _guard = self.send_lock.lock().await;

        let count = self.can_message_count.fetch_add(1, Ordering::SeqCst) + 1;

        let frequency = {
            let mut last_time = self.last_message_time.lock().unwrap();
            let now = Instant::now();
            let duration = now.duration_since(*last_time);
            *last_time = now;
            let frequency = 1.0 / duration.as_secs_f64();
            frequency
        };

        if self.debug_mode {
            debug!(
                "[{} / {:.2} Hz] [{}]: {:?}",
                count, frequency, actuator_id, message
            );
            sleep(Duration::from_millis(50)).await;
        } else {
            // TODO: Implement CAN message sending.
            info!(
                "[{} / {:.2} Hz] [{}]: {:?}",
                count, frequency, actuator_id, message
            );
        }
    }

    async fn send_position_messages(&self) {
        let positions: HashMap<u8, f64> = self.target_positions.lock().unwrap().clone();
        for (actuator_id, _position) in positions {
            self.send_can_message(actuator_id, [0u8; 8]).await;
        }
    }

    async fn send_pid_params_messages(&self) {
        let params: HashMap<u8, PIDParams> = self.pid_params.lock().unwrap().clone();
        if params.is_empty() {
            return;
        }
        for (actuator_id, params) in params {
            self.send_pid_params_message(actuator_id, params).await;
        }
        self.pid_params.lock().unwrap().clear();
    }

    async fn send_pid_params_message(&self, actuator_id: u8, params: PIDParams) {
        let params_bytes = [
            map_float_to_u8(params.pos_kp, 0.0, 1.0),
            map_float_to_u8(params.pos_kd, 0.0, 1.0),
            map_float_to_u8(params.vel_kp, 0.0, 1.0),
            map_float_to_u8(params.vel_kd, 0.0, 1.0),
            map_float_to_u8(params.tor_kp, 0.0, 1.0),
            map_float_to_u8(params.tor_kd, 0.0, 1.0),
            0u8,
            0u8,
        ];
        self.send_can_message(actuator_id, params_bytes).await;
    }
}

impl ActuatorImpl for Robstride {
    fn set_position(&self, actuator_id: u8, position: f64) {
        let mut positions = self.target_positions.lock().unwrap();
        positions.insert(actuator_id, position);
    }

    fn get_position(&self, actuator_id: u8) -> f64 {
        self.target_positions
            .lock()
            .unwrap()
            .get(&actuator_id)
            .cloned()
            .unwrap_or(0.0)
    }

    fn update(&self) -> Pin<Box<dyn Future<Output = ()> + Send + '_>> {
        Box::pin(async move {
            self.send_position_messages().await;
            self.send_pid_params_messages().await;
        })
    }

    fn set_pid_params(
        &self,
        actuator_id: u8,
        pos_kp: f64,
        pos_kd: f64,
        vel_kp: f64,
        vel_kd: f64,
        tor_kp: f64,
        tor_kd: f64,
    ) {
        let mut pid_params = self.pid_params.lock().unwrap();
        pid_params.insert(
            actuator_id,
            PIDParams {
                pos_kp,
                pos_kd,
                vel_kp,
                vel_kd,
                tor_kp,
                tor_kd,
            },
        );
    }

    fn get_pid_params(&self, actuator_id: u8) -> (f64, f64, f64, f64, f64, f64) {
        let pid_params = self.pid_params.lock().unwrap();
        let params = pid_params.get(&actuator_id).cloned();
        match params {
            Some(params) => (
                params.pos_kp,
                params.pos_kd,
                params.vel_kp,
                params.vel_kd,
                params.tor_kp,
                params.tor_kd,
            ),
            None => (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        }
    }
}

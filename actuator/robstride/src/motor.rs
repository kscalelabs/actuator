use crate::can::{float_to_uint, rx_unpack, tx_packs, uint_to_float, unpack_raw_feedback, ExId};
use crate::can::{CanComMode, CanPack};
use crate::config::MotorConfig;
use crate::config::ROBSTRIDE_CONFIGS;
use crate::port::init_serial_port;
use crate::types::{MotorMode, MotorType, RunMode};
use serde::{Deserialize, Serialize};
use serialport::TTYPort;
use std::collections::HashMap;
use std::thread;
use std::time::Duration;

pub const CAN_ID_DEBUG_UI: u8 = 0xFD;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct MotorControlParams {
    pub position: f32,
    pub velocity: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque: f32,
}

impl Default for MotorControlParams {
    fn default() -> Self {
        MotorControlParams {
            position: 0.0,
            velocity: 0.0,
            kp: 0.0,
            kd: 0.0,
            torque: 0.0,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MotorSdoParams {
    pub torque_limit: Option<f32>,
    pub speed_limit: Option<f32>,
    pub current_limit: Option<f32>,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct MotorFeedback {
    pub can_id: u8,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mode: MotorMode,
    pub faults: u16,
}

pub struct Motors {
    pub port: TTYPort,
    pub motor_configs: HashMap<u8, &'static MotorConfig>,
    pub mode: RunMode,
    pub sleep_time: Duration,
    pub verbose: bool,
}

impl Motors {
    pub fn new(
        port_name: &str,
        motor_infos: &HashMap<u8, MotorType>,
        verbose: bool,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        if motor_infos.is_empty() {
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "No motors to initialize",
            )));
        }

        let port = init_serial_port(port_name)?;
        let motor_configs: HashMap<u8, &'static MotorConfig> = motor_infos
            .clone()
            .into_iter()
            .filter_map(|(id, motor_type)| {
                ROBSTRIDE_CONFIGS
                    .get(&motor_type)
                    .map(|config| (id, config))
            })
            .collect();

        Ok(Motors {
            port,
            motor_configs,
            mode: RunMode::UnsetMode,
            sleep_time: Duration::from_millis(20),
            verbose,
        })
    }

    fn send_command(&mut self, pack: &CanPack, sleep_after: bool) -> std::io::Result<CanPack> {
        tx_packs(&mut self.port, &[pack.clone()], self.verbose)?;
        if sleep_after {
            thread::sleep(self.sleep_time);
        }
        let packs = rx_unpack(&mut self.port, 1, self.verbose)?;
        if packs.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Failed to receive CAN packet",
            ));
        }
        if packs.len() > 1 {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Received multiple CAN packets",
            ));
        }
        Ok(packs[0].clone())
    }

    fn send_commands(
        &mut self,
        packs: &[CanPack],
        sleep_after: bool,
        serial: bool,
    ) -> std::io::Result<Vec<CanPack>> {
        if packs.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "No commands to send!",
            ));
        }
        if serial {
            let mut results = Vec::new();
            for pack in packs {
                results.push(self.send_command(pack, sleep_after)?);
            }
            Ok(results)
        } else {
            tx_packs(&mut self.port, packs, self.verbose)?;
            if sleep_after {
                thread::sleep(self.sleep_time);
            }
            rx_unpack(&mut self.port, packs.len(), self.verbose)
        }
    }

    pub fn send_get_mode(&mut self) -> Result<HashMap<u8, RunMode>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut modes = HashMap::new();

        for id in motor_ids {
            let mut pack = CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::SdoRead,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            };

            let index: u16 = 0x7005;
            pack.data[..2].copy_from_slice(&index.to_le_bytes());

            match self.send_command(&pack, false) {
                Ok(response) => {
                    let mode = unsafe { std::mem::transmute(response.data[4] as u8) };
                    modes.insert(id, mode);
                }
                Err(err) => {
                    return Err(err);
                }
            }
        }

        Ok(modes)
    }

    pub fn send_set_mode(
        &mut self,
        mode: RunMode,
    ) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        if self.mode == RunMode::UnsetMode {
            let read_mode = self.send_get_mode()?;
            if read_mode.is_empty() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Other,
                    "Failed to get the current mode",
                ));
            }

            let single_read_mode = read_mode.values().next().unwrap().clone();
            if read_mode.values().all(|&x| x == single_read_mode) {
                self.mode = single_read_mode;
            }
        }

        if self.mode == mode {
            return Ok(HashMap::new());
        }

        self.mode = mode;

        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut feedbacks = HashMap::new();

        for id in motor_ids {
            let mut pack = CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::SdoWrite,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            };

            let index: u16 = 0x7005;
            pack.data[..2].copy_from_slice(&index.to_le_bytes());
            pack.data[4] = mode as u8;

            match self.send_command(&pack, true) {
                Ok(pack) => {
                    let feedback = self.unpack_feedback(&pack)?;
                    feedbacks.insert(id, feedback);
                }
                Err(_) => continue,
            }
        }

        Ok(feedbacks)
    }

    pub fn send_set_zero(&mut self, motor_id: u8) -> Result<(), std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorZero,
                res: 0,
            },
            len: 8,
            data: vec![1, 0, 0, 0, 0, 0, 0, 0],
        };

        self.send_command(&pack, true)?;
        Ok(())
    }

    pub fn send_set_zeros(&mut self, motor_ids: Option<&[u8]>) -> Result<(), std::io::Error> {
        let ids_to_zero = motor_ids
            .map(|ids| ids.to_vec())
            .unwrap_or_else(|| self.motor_configs.keys().cloned().collect());

        for &id in &ids_to_zero {
            if !self.motor_configs.contains_key(&id) {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Invalid motor ID: {}", id),
                ));
            }
        }

        // Zero.
        for &id in &ids_to_zero {
            self.send_set_zero(id)?;
        }

        Ok(())
    }

    pub fn zero_motors(&mut self, motor_ids: &[u8]) -> Result<(), std::io::Error> {
        self.send_zero_torque(motor_ids)?;
        self.send_set_zeros(Some(motor_ids))?;
        Ok(())
    }

    fn read_u16_param(&mut self, motor_id: u8, index: u16) -> Result<u16, std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::ParaRead,
                res: 0,
            },
            len: 8,
            data: vec![0; 8],
        };

        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        tx_packs(&mut self.port, &[pack], self.verbose)?;

        let packs = rx_unpack(&mut self.port, 1, self.verbose)?;
        if packs.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Failed to receive CAN packet",
            ));
        }
        let pack = packs[0].clone();

        Ok(u16::from_le_bytes([pack.data[4], pack.data[5]]))
    }

    pub fn read_can_timeout(&mut self, id: u8) -> Result<f32, std::io::Error> {
        let config = *self.motor_configs.get(&id).ok_or(std::io::Error::new(
            std::io::ErrorKind::NotFound,
            "Motor not found",
        ))?;
        let timeout = self.read_u16_param(id, config.can_timeout_command)?;
        Ok(timeout as f32 * 1000.0 / config.can_timeout_factor) // In milliseconds
    }

    pub fn read_can_timeouts(&mut self) -> Result<HashMap<u8, f32>, std::io::Error> {
        let mut timeouts = HashMap::new();

        for (&id, config) in self.motor_configs.clone().iter() {
            let timeout = self.read_u16_param(id, config.can_timeout_command)?;
            timeouts.insert(id, timeout as f32 * 1000.0 / config.can_timeout_factor);
        }
        Ok(timeouts)
    }

    pub fn send_can_timeout(&mut self, timeout: f32) -> Result<(), std::io::Error> {
        // NOTE: This function does not seem to work very reliabliy.
        for (&id, config) in self.motor_configs.clone().iter() {
            let mut pack = CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::ParaWrite,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            };

            let index: u16 = config.can_timeout_command;
            pack.data[..2].copy_from_slice(&index.to_le_bytes());
            pack.data[2] = 0x04;

            let new_timeout = (timeout * config.can_timeout_factor / 1000.0)
                .round()
                .clamp(0.0, 100000.0) as u32;
            println!("Setting CAN timeout to {} for motor {}", new_timeout, id);
            pack.data[4..8].copy_from_slice(&new_timeout.to_le_bytes());

            let _ = self.send_command(&pack, true);

            println!("CAN timeout set to {}", new_timeout);
        }

        Ok(())
    }

    fn write_sdo_param(&mut self, motor_id: u8, index: u16, value: f32) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite,
                res: 0,
            },
            len: 8,
            data: vec![0; 8],
        };
        
        pack.data[..2].copy_from_slice(&index.to_le_bytes());

        pack.data[4..8].copy_from_slice(&value.to_le_bytes());

        self.send_command(&pack, true)?;
        Ok(())  
    }

    // Set speed limit in rad/s
    pub fn set_speed_limit(&mut self, motor_id: u8, speed_limit: f32) -> Result<(), std::io::Error> {
        let index: u16 = 0x7017;
        self.write_sdo_param(motor_id, index, speed_limit)?;
        Ok(())
    }

    // Set current limit in A
    pub fn set_current_limit(&mut self, motor_id: u8, current_limit: f32) -> Result<(), std::io::Error> {
        let index: u16 = 0x7018;
        self.write_sdo_param(motor_id, index, current_limit)?;
        Ok(())
    }

    // Set torque limit in Nm
    pub fn set_torque_limit(&mut self, motor_id: u8, torque_limit: f32) -> Result<(), std::io::Error> {
        let index: u16 = 0x700B;
        // optionally clamp to min/max
        self.write_sdo_param(motor_id, index, torque_limit)?;
        Ok(())
    }

    pub fn send_reset(&mut self, id: u8) -> Result<CanPack, std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorReset,
                res: 0,
            },
            len: 8,
            data: vec![0; 8],
        };

        self.send_command(&pack, true)
    }

    pub fn send_resets(&mut self) -> Result<(), std::io::Error> {
        let motor_ids: Vec<u8> = self.motor_configs.keys().cloned().collect();
        for id in motor_ids {
            self.send_reset(id)?;
        }
        Ok(())
    }

    pub fn send_start(&mut self, id: u8) -> Result<CanPack, std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorIn,
                res: 0,
            },
            len: 8,
            data: vec![0; 8],
        };

        self.send_command(&pack, true)
    }

    pub fn send_starts(&mut self) -> Result<(), std::io::Error> {
        let motor_ids: Vec<u8> = self.motor_configs.keys().cloned().collect();
        for id in motor_ids {
            self.send_start(id)?;
        }
        Ok(())
    }

    fn pack_motor_params(
        &mut self,
        id: u8,
        params: &MotorControlParams,
    ) -> Result<CanPack, std::io::Error> {
        if let Some(config) = self.motor_configs.get(&id) {
            let mut pack = CanPack {
                ex_id: ExId {
                    id,
                    data: 0,
                    mode: CanComMode::MotorCtrl,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            };

            let pos_int_set = float_to_uint(params.position, config.p_min, config.p_max, 16);
            let vel_int_set = float_to_uint(params.velocity, config.v_min, config.v_max, 16);
            let kp_int_set = float_to_uint(params.kp, config.kp_min, config.kp_max, 16);
            let kd_int_set = float_to_uint(params.kd, config.kd_min, config.kd_max, 16);
            let torque_int_set = float_to_uint(params.torque, config.t_min, config.t_max, 16);

            pack.ex_id.data = torque_int_set;

            pack.data[0..2].copy_from_slice(&pos_int_set.to_be_bytes());
            pack.data[2..4].copy_from_slice(&vel_int_set.to_be_bytes());
            pack.data[4..6].copy_from_slice(&kp_int_set.to_be_bytes());
            pack.data[6..8].copy_from_slice(&kd_int_set.to_be_bytes());

            Ok(pack)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "Motor not found",
            ))
        }
    }

    pub fn send_motor_controls(
        &mut self,
        params_map: &HashMap<u8, MotorControlParams>,
        serial: bool,
    ) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        self.send_set_mode(RunMode::MitMode)?;

        let packs: Vec<CanPack> = params_map
            .iter()
            .filter_map(|(&id, params)| self.pack_motor_params(id, params).ok())
            .collect();

        if packs.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "No motor control parameters provided",
            ));
        }

        let response_packs = if serial {
            packs
                .into_iter()
                .map(|pack| self.send_command(&pack, false))
                .collect::<Result<Vec<_>, _>>()?
        } else {
            self.send_commands(&packs, false, false)?
        };

        Ok(response_packs
            .into_iter()
            .filter_map(|pack| {
                self.unpack_feedback(&pack)
                    .map(|feedback| (feedback.can_id, feedback))
                    .ok()
            })
            .collect::<HashMap<u8, MotorFeedback>>())
    }

    pub fn send_zero_torque(&mut self, motor_ids: &[u8]) -> Result<(), std::io::Error> {
        let params = HashMap::from_iter(
            motor_ids
                .iter()
                .map(|&id| (id, MotorControlParams::default())),
        );
        self.send_motor_controls(&params, true)?;
        Ok(())
    }

    fn unpack_feedback(&mut self, pack: &CanPack) -> Result<MotorFeedback, std::io::Error> {
        let raw_feedback = unpack_raw_feedback(pack);

        if let Some(config) = self.motor_configs.get(&raw_feedback.can_id) {
            let position = uint_to_float(raw_feedback.pos_int, config.p_min, config.p_max, 16);
            let velocity = uint_to_float(raw_feedback.vel_int, config.v_min, config.v_max, 16);
            let torque = uint_to_float(raw_feedback.torque_int, config.t_min, config.t_max, 16);

            Ok(MotorFeedback {
                can_id: raw_feedback.can_id,
                position,
                velocity,
                torque,
                mode: raw_feedback.mode,
                faults: raw_feedback.faults,
            })
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "Motor not found",
            ))
        }
    }
}

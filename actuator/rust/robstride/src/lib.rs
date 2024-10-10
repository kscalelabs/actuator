use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{Read, Write};
use std::time::Duration;

#[macro_use]
extern crate lazy_static;

pub const CAN_ID_MASTER: u8 = 0x00;
pub const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F;
pub const CAN_ID_BROADCAST: u8 = 0xFE;
pub const CAN_ID_DEBUG_UI: u8 = 0xFD;

pub const BAUDRATE: u32 = 921600;

pub struct MotorConfig {
    pub p_min: f32,
    pub p_max: f32,
    pub v_min: f32,
    pub v_max: f32,
    pub kp_min: f32,
    pub kp_max: f32,
    pub kd_min: f32,
    pub kd_max: f32,
    pub t_min: f32,
    pub t_max: f32,
}

lazy_static! {
    pub static ref ROBSTRIDE_CONFIGS: HashMap<MotorType, MotorConfig> = {
        let mut m = HashMap::new();
        m.insert(
            MotorType::Type01,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -44.0,
                v_max: 44.0,
                kp_min: 0.0,
                kp_max: 500.0,
                kd_min: 0.0,
                kd_max: 5.0,
                t_min: -12.0,
                t_max: 12.0,
            },
        );
        m.insert(
            MotorType::Type03,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -20.0,
                v_max: 20.0,
                kp_min: 0.0,
                kp_max: 5000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                t_min: -60.0,
                t_max: 60.0,
            },
        );
        m.insert(
            MotorType::Type04,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -15.0,
                v_max: 15.0,
                kp_min: 0.0,
                kp_max: 5000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                t_min: -120.0,
                t_max: 120.0,
            },
        );
        m
    };
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum CanComMode {
    AnnounceDevId = 0,
    MotorCtrl,
    MotorFeedback,
    MotorIn,
    MotorReset,
    MotorCali,
    MotorZero,
    MotorId,
    ParaWrite,
    ParaRead,
    ParaUpdate,
    OtaStart,
    OtaInfo,
    OtaIng,
    OtaEnd,
    CaliIng,
    CaliRst,
    SdoRead,
    SdoWrite,
    ParaStrInfo,
    MotorBrake,
    FaultWarn,
    ModeTotal,
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, Default)]
pub enum MotorMode {
    #[default]
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum RunMode {
    UnsetMode = -1,
    MitMode = 0,
    PositionMode = 1,
    SpeedMode = 2,
    CurrentMode = 3,
    ToZeroMode = 4,
    CspPositionMode = 5,
}

pub struct ExId {
    pub id: u8,
    pub data: u16,
    pub mode: CanComMode,
    pub res: u8,
}

pub struct CanPack {
    pub ex_id: ExId,
    pub len: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Default, Clone)]
pub struct MotorFeedback {
    pub can_id: u8,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mode: MotorMode,
    pub faults: u16,
}

#[derive(Debug, Default, Clone)]
pub struct MotorFeedbackRaw {
    pub can_id: u8,
    pub pos_int: u16,
    pub vel_int: u16,
    pub torque_int: u16,
    pub mode: MotorMode,
    pub faults: u16,
}

fn init_serial_port(device: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
    let port = serialport::new(device, BAUDRATE)
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .timeout(Duration::from_millis(10))
        .open()?;
    Ok(port)
}

fn pack_bits(values: &[u32], bit_lengths: &[u8]) -> u32 {
    let mut result: u32 = 0;
    let mut current_shift = 0;

    for (&value, &bits) in values.iter().zip(bit_lengths.iter()).rev() {
        let mask = (1 << bits) - 1;
        result |= (value & mask) << current_shift;
        current_shift += bits;
    }

    result
}

fn uint_to_float(x_int: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span = x_max - x_min;
    let offset = x_min;
    (x_int as f32) * span / ((1 << bits) - 1) as f32 + offset
}

fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let offset = x_min;
    ((x - offset) * ((1 << bits) - 1) as f32 / span) as u16
}

fn tx_pack(port: &mut Box<dyn SerialPort>, pack: &CanPack) -> Result<(), std::io::Error> {
    let mut buffer = Vec::new();
    buffer.extend_from_slice(b"AT");

    let addr = (pack_bits(
        &[
            pack.ex_id.res as u32,
            pack.ex_id.mode as u32,
            pack.ex_id.data as u32,
            pack.ex_id.id as u32,
        ],
        &[3, 5, 16, 8],
    ) << 3)
        | 0x00000004;

    buffer.extend_from_slice(&addr.to_be_bytes());
    buffer.push(pack.len);
    buffer.extend_from_slice(&pack.data[..pack.len as usize]);
    buffer.extend_from_slice(b"\r\n");

    port.write_all(&buffer)?;
    port.flush()?;
    Ok(())
}

fn rx_unpack(port: &mut Box<dyn SerialPort>) -> Result<CanPack, std::io::Error> {
    let mut buffer = [0u8; 17];
    let bytes_read = port.read(&mut buffer)?;

    if bytes_read == 17 && buffer[0] == b'A' && buffer[1] == b'T' {
        let addr = u32::from_be_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]) >> 3;
        let ex_id = ExId {
            id: (addr & 0xFF) as u8,
            data: ((addr >> 8) & 0xFFFF) as u16,
            mode: unsafe { std::mem::transmute((addr >> 24) as u8) },
            res: 0,
        };

        Ok(CanPack {
            ex_id,
            len: buffer[6],
            data: buffer[7..16].to_vec(),
        })
    } else {
        Err(std::io::Error::new(
            std::io::ErrorKind::UnexpectedEof,
            "Failed to read CAN packet",
        ))
    }
}

fn rx_unpack_feedback(port: &mut Box<dyn SerialPort>) -> Result<MotorFeedbackRaw, std::io::Error> {
    match rx_unpack(port) {
        Ok(pack) => {
            let can_id = (pack.ex_id.data & 0x00FF) as u8;
            let faults = (pack.ex_id.data & 0x3F00) >> 8;
            let mode = unsafe { std::mem::transmute(((pack.ex_id.data & 0xC000) >> 14) as u8) };

            let pos_int = u16::from_be_bytes([pack.data[0], pack.data[1]]);
            let vel_int = u16::from_be_bytes([pack.data[2], pack.data[3]]);
            let torque_int = u16::from_be_bytes([pack.data[4], pack.data[5]]);

            Ok(MotorFeedbackRaw {
                can_id,
                pos_int,
                vel_int,
                torque_int,
                mode,
                faults,
            })
        }
        Err(_) => Ok(MotorFeedbackRaw::default()),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotorType {
    Type01,
    Type03,
    Type04,
}

pub struct MotorInfo {
    pub id: u8,
    pub motor_type: MotorType,
}

pub struct Motors {
    port: Box<dyn SerialPort>,
    motor_configs: HashMap<u8, &'static MotorConfig>,
    latest_feedback: HashMap<u8, MotorFeedback>,
    pending_responses: usize,
    mode: RunMode,
    sleep_time: Duration,
}

impl Motors {
    pub fn new(
        port_name: &str,
        motor_infos: Vec<MotorInfo>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let port = init_serial_port(port_name)?;
        let motor_configs: HashMap<u8, &'static MotorConfig> = motor_infos
            .into_iter()
            .filter_map(|info| {
                ROBSTRIDE_CONFIGS
                    .get(&info.motor_type)
                    .map(|config| (info.id, config))
            })
            .collect();

        Ok(Motors {
            port,
            motor_configs,
            latest_feedback: HashMap::new(),
            pending_responses: 0,
            mode: RunMode::UnsetMode,
            sleep_time: Duration::from_millis(50),
        })
    }

    fn send_command(&mut self, pack: &CanPack) -> Result<(), std::io::Error> {
        tx_pack(&mut self.port, pack)?;
        self.pending_responses += 1;
        Ok(())
    }

    pub fn send_get_mode(&mut self) -> Result<HashMap<u8, RunMode>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();

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
            tx_pack(&mut self.port, &pack)?;
        }

        match rx_unpack(&mut self.port) {
            Ok(pack) => {
                let mode = unsafe { std::mem::transmute(pack.data[4] as u8) };
                Ok(HashMap::from([(pack.ex_id.id, mode)]))
            }
            Err(_) => Ok(HashMap::new()),
        }
    }

    fn send_set_mode(
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
            self.send_command(&pack)?;
        }

        // After setting the mode for all motors, sleep for a short time.
        std::thread::sleep(self.sleep_time);

        self.read_all_pending_responses()
    }

    pub fn send_set_zero(&mut self) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();

        for id in motor_ids {
            let pack = CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::MotorZero,
                    res: 0,
                },
                len: 8,
                data: vec![1, 0, 0, 0, 0, 0, 0, 0],
            };

            self.send_command(&pack)?;
        }

        // After setting the mode for all motors, sleep for a short time.
        std::thread::sleep(self.sleep_time);

        self.read_all_pending_responses()
    }

    pub fn send_reset(&mut self) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();

        for id in motor_ids {
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

            self.send_command(&pack)?;
        }

        // After sending the reset command, sleep for a short time.
        std::thread::sleep(self.sleep_time);
        self.read_all_pending_responses()
    }

    pub fn send_start(&mut self) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();

        for id in motor_ids {
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

            self.send_command(&pack)?;
        }

        // After sending the start command, sleep for a short time.
        std::thread::sleep(self.sleep_time);
        self.read_all_pending_responses()
    }

    fn send_motor_control(
        &mut self,
        id: u8,
        pos_set: f32,
        vel_set: f32,
        kp_set: f32,
        kd_set: f32,
        torque_set: f32,
    ) -> Result<(), std::io::Error> {
        self.send_set_mode(RunMode::MitMode)?;

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

            let pos_int_set = float_to_uint(pos_set, config.p_min, config.p_max, 16);
            let vel_int_set = float_to_uint(vel_set, config.v_min, config.v_max, 16);
            let kp_int_set = float_to_uint(kp_set, config.kp_min, config.kp_max, 16);
            let kd_int_set = float_to_uint(kd_set, config.kd_min, config.kd_max, 16);
            let torque_int_set = float_to_uint(torque_set, config.t_min, config.t_max, 16);

            pack.ex_id.data = torque_int_set;

            pack.data[0] = (pos_int_set >> 8) as u8;
            pack.data[1] = (pos_int_set & 0xFF) as u8;
            pack.data[2] = (vel_int_set >> 8) as u8;
            pack.data[3] = (vel_int_set & 0xFF) as u8;
            pack.data[4] = (kp_int_set >> 8) as u8;
            pack.data[5] = (kp_int_set & 0xFF) as u8;
            pack.data[6] = (kd_int_set >> 8) as u8;
            pack.data[7] = (kd_int_set & 0xFF) as u8;

            self.send_command(&pack)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "Motor not found",
            ))
        }
    }

    fn send_torque_control(&mut self, motor_id: u8, torque_set: f32) -> Result<(), std::io::Error> {
        self.send_motor_control(motor_id, 0.0, 0.0, 0.0, 0.0, torque_set)
    }

    pub fn send_torque_controls(
        &mut self,
        torque_sets: &HashMap<u8, f32>,
    ) -> Result<HashMap<u8, MotorFeedback>, Box<dyn std::error::Error>> {
        // Check if all provided motor IDs are valid
        for &motor_id in torque_sets.keys() {
            if !self.motor_configs.contains_key(&motor_id) {
                return Err(Box::new(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Invalid motor ID: {}", motor_id),
                )));
            }
        }

        // Send torque commands for each motor
        for (&motor_id, &torque_set) in torque_sets {
            self.send_torque_control(motor_id, torque_set)?;
        }

        self.read_all_pending_responses().map_err(|e| e.into())
    }

    fn read_all_pending_responses(&mut self) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        while self.pending_responses > 0 {
            match rx_unpack_feedback(&mut self.port) {
                Ok(raw_feedback) => {
                    if let Some(config) = self.motor_configs.get(&raw_feedback.can_id) {
                        let position =
                            uint_to_float(raw_feedback.pos_int, config.p_min, config.p_max, 16);
                        let velocity =
                            uint_to_float(raw_feedback.vel_int, config.v_min, config.v_max, 16);
                        let torque =
                            uint_to_float(raw_feedback.torque_int, config.t_min, config.t_max, 16);

                        let feedback = MotorFeedback {
                            can_id: raw_feedback.can_id,
                            position,
                            velocity,
                            torque,
                            mode: raw_feedback.mode,
                            faults: raw_feedback.faults,
                        };

                        self.latest_feedback.insert(raw_feedback.can_id, feedback);
                    }
                    self.pending_responses -= 1;
                }
                Err(_) => {
                    self.pending_responses -= 1;
                }
            }
        }
        Ok(self.latest_feedback.clone())
    }

    pub fn get_latest_feedback(&self) -> HashMap<u8, MotorFeedback> {
        self.latest_feedback.clone()
    }

    pub fn get_latest_feedback_for(&self, motor_id: u8) -> Result<&MotorFeedback, std::io::Error> {
        self.latest_feedback
            .get(&motor_id)
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, "No feedback found"))
    }
}

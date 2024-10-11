use serialport::SerialPort;
use std::collections::{HashMap, HashSet};
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::thread;
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
    pub kp_default: f32,
    pub kd_min: f32,
    pub kd_max: f32,
    pub kd_default: f32,
    pub t_min: f32,
    pub t_max: f32,
    pub zero_on_init: bool,
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
                kp_default: 20.0,
                kd_min: 0.0,
                kd_max: 5.0,
                kd_default: 1.0,
                t_min: -12.0,
                t_max: 12.0,
                zero_on_init: true, // Single encoder motor.
            },
        );
        // This is probably not correct, the Type02 is not released yet.
        m.insert(
            MotorType::Type02,
            MotorConfig {
                p_min: -12.5,
                p_max: 12.5,
                v_min: -44.0,
                v_max: 44.0,
                kp_min: 0.0,
                kp_max: 500.0,
                kp_default: 10.0,
                kd_min: 0.0,
                kd_max: 5.0,
                kd_default: 1.0,
                t_min: -12.0,
                t_max: 12.0,
                zero_on_init: false,
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
                kp_default: 1000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                kd_default: 10.0,
                t_min: -60.0,
                t_max: 60.0,
                zero_on_init: false,
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
                kp_default: 1000.0,
                kd_min: 0.0,
                kd_max: 100.0,
                kd_default: 10.0,
                t_min: -120.0,
                t_max: 120.0,
                zero_on_init: false,
            },
        );
        m
    };
}

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
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

#[derive(Debug, Clone)]
pub struct ExId {
    pub id: u8,
    pub data: u16,
    pub mode: CanComMode,
    pub res: u8,
}

#[derive(Debug, Clone)]
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

    // println!(
    //     "TX: {}",
    //     buffer
    //         .iter()
    //         .map(|b| format!("{:02X}", b))
    //         .collect::<Vec<String>>()
    //         .join(" ")
    // );

    port.write_all(&buffer)?;
    port.flush()?;
    Ok(())
}

fn rx_unpack(port: &mut Box<dyn SerialPort>) -> Result<CanPack, std::io::Error> {
    let mut buffer = [0u8; 7];
    port.read_exact(&mut buffer)?;

    // print!(
    //     "RX: {} ",
    //     buffer
    //         .iter()
    //         .map(|b| format!("{:02X}", b))
    //         .collect::<Vec<String>>()
    //         .join(" ")
    // );

    if buffer[0] == b'A' && buffer[1] == b'T' {
        let addr = u32::from_be_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]) >> 3;
        let ex_id = ExId {
            id: (addr & 0xFF) as u8,
            data: ((addr >> 8) & 0xFFFF) as u16,
            mode: unsafe { std::mem::transmute((addr >> 24) as u8) },
            res: 0,
        };
        let len = buffer[6];
        let mut buffer = vec![0u8; len as usize + 2];
        port.read_exact(&mut buffer)?;

        // println!(
        //     "{}",
        //     buffer
        //         .iter()
        //         .map(|b| format!("{:02X}", b))
        //         .collect::<Vec<String>>()
        //         .join(" ")
        // );

        Ok(CanPack {
            ex_id,
            len,
            data: buffer,
        })
    } else {
        println!("Failed to read CAN packet");

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
    Type02,
    Type03,
    Type04,
}

pub fn motor_type_from_str(s: &str) -> Result<MotorType, std::io::Error> {
    match s {
        "01" => Ok(MotorType::Type01),
        "02" => Ok(MotorType::Type02),
        "03" => Ok(MotorType::Type03),
        "04" => Ok(MotorType::Type04),
        _ => Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "Invalid motor type",
        )),
    }
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
        motor_infos: &HashMap<u8, MotorType>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
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

    pub fn send_set_zero(
        &mut self,
        motor_ids: Option<&[u8]>,
    ) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
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

        // Reset.
        for &id in &ids_to_zero {
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
        std::thread::sleep(self.sleep_time);

        // Zero.
        for &id in &ids_to_zero {
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
        std::thread::sleep(self.sleep_time);

        // Start.
        for &id in &ids_to_zero {
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
        std::thread::sleep(self.sleep_time);

        self.read_all_pending_responses()
    }

    fn read_string_param(
        &mut self,
        motor_id: u8,
        index: u16,
        num_packs: u8,
    ) -> Result<String, std::io::Error> {
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

        let index: u16 = index;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        tx_pack(&mut self.port, &pack)?;

        let mut packs = Vec::new();
        for _ in 0..num_packs {
            packs.push(rx_unpack(&mut self.port)?);
        }

        let name = packs
            .iter()
            .flat_map(|pack| pack.data[4..8].iter())
            .map(|&b| b as char)
            .filter(|&c| c != '\0') // Filter out null characters
            .collect::<String>();
        Ok(name)
    }

    fn read_uint_param(&mut self, motor_id: u8, index: u16) -> Result<u32, std::io::Error> {
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

        let index: u16 = index;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        tx_pack(&mut self.port, &pack)?;

        let pack = rx_unpack(&mut self.port)?;
        let value = u32::from_le_bytes(pack.data[4..8].try_into().unwrap());
        Ok(value)
    }

    pub fn read_names(&mut self) -> Result<HashMap<u8, String>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut names = HashMap::new();

        for id in motor_ids {
            let name = self.read_string_param(id, 0x0000, 4)?;
            names.insert(id, name);
        }
        Ok(names)
    }

    pub fn read_bar_codes(&mut self) -> Result<HashMap<u8, String>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut names = HashMap::new();

        for id in motor_ids {
            let name = self.read_string_param(id, 0x0001, 4)?;
            names.insert(id, name);
        }
        Ok(names)
    }

    pub fn read_build_dates(&mut self) -> Result<HashMap<u8, String>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut names = HashMap::new();

        for id in motor_ids {
            let name = self.read_string_param(id, 0x1001, 3)?;
            names.insert(id, name);
        }

        Ok(names)
    }

    pub fn read_can_timeout(&mut self) -> Result<HashMap<u8, u32>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();
        let mut timeouts = HashMap::new();

        for id in motor_ids {
            let timeout = self.read_uint_param(id, 0x200c)?;
            timeouts.insert(id, timeout);
        }
        Ok(timeouts)
    }

    pub fn send_can_timeout(
        &mut self,
        timeout: u32,
    ) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        let motor_ids = self.motor_configs.keys().cloned().collect::<Vec<u8>>();

        for id in motor_ids {
            let mut pack = CanPack {
                ex_id: ExId {
                    id: id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::ParaWrite,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            };

            let index: u16 = 0x200c;
            pack.data[..2].copy_from_slice(&index.to_le_bytes());
            pack.data[2] = 0x04;

            // Convert milliseconds to correct unit by multiplying by 20.
            // Set to zero to disable timeout.
            let timeout = (timeout * 20).clamp(0, 100000);
            pack.data[4..8].copy_from_slice(&timeout.to_le_bytes());

            self.send_command(&pack)?;
        }

        // After sending the reset command, sleep for a short time.
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
            pack.data[0..2].copy_from_slice(&pos_int_set.to_be_bytes());
            pack.data[2..4].copy_from_slice(&vel_int_set.to_be_bytes());
            pack.data[4..6].copy_from_slice(&kp_int_set.to_be_bytes());
            pack.data[6..8].copy_from_slice(&kd_int_set.to_be_bytes());

            self.send_command(&pack)
        } else {
            Err(std::io::Error::new(
                std::io::ErrorKind::NotFound,
                "Motor not found",
            ))
        }
    }

    fn send_pd_control(
        &mut self,
        motor_id: u8,
        pos_set: f32,
        vel_set: f32,
        kp_set: f32,
        kd_set: f32,
    ) -> Result<(), std::io::Error> {
        self.send_motor_control(motor_id, pos_set, vel_set, kp_set, kd_set, 0.0)
    }

    pub fn send_pd_controls(
        &mut self,
        pd_sets: &HashMap<u8, (f32, f32, f32, f32)>,
    ) -> Result<HashMap<u8, MotorFeedback>, Box<dyn std::error::Error>> {
        // Check if all provided motor IDs are valid
        for &motor_id in pd_sets.keys() {
            if !self.motor_configs.contains_key(&motor_id) {
                return Err(Box::new(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Invalid motor ID: {}", motor_id),
                )));
            }
        }

        // Send PD commands for each motor
        for (&motor_id, &(pos_set, vel_set, kp, kd)) in pd_sets {
            self.send_pd_control(motor_id, pos_set, vel_set, kp, kd)?;
        }
        self.read_all_pending_responses().map_err(|e| e.into())
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

pub struct MotorsSupervisor {
    motors: Arc<Mutex<Motors>>,
    target_positions: Arc<Mutex<HashMap<u8, f32>>>,
    kp_kd_values: Arc<Mutex<HashMap<u8, (f32, f32)>>>,
    running: Arc<Mutex<bool>>,
    latest_feedback: Arc<Mutex<HashMap<u8, MotorFeedback>>>,
    motors_to_zero: Arc<Mutex<HashSet<u8>>>,
    sleep_duration: Arc<Mutex<Duration>>,
    paused: Arc<Mutex<bool>>,
}

impl MotorsSupervisor {
    pub fn new(
        port_name: &str,
        motor_infos: &HashMap<u8, MotorType>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        // Initialize Motors
        let motors = Motors::new(port_name, motor_infos)?;

        // Get default KP/KD values for all motors.
        let kp_kd_values = motors
            .motor_configs
            .iter()
            .map(|(id, config)| (*id, (config.kp_default, config.kd_default)))
            .collect::<HashMap<u8, (f32, f32)>>();

        // Find motors that need to be zeroed on initialization.
        let zero_on_init_motors = motors
            .motor_configs
            .iter()
            .filter(|(_, &config)| config.zero_on_init)
            .map(|(&id, _)| id)
            .collect::<HashSet<u8>>();

        let motors = Arc::new(Mutex::new(motors));
        let motors_to_zero = Arc::new(Mutex::new(zero_on_init_motors));
        let target_positions = Arc::new(Mutex::new(HashMap::new()));
        let kp_kd_values = Arc::new(Mutex::new(kp_kd_values));
        let running = Arc::new(Mutex::new(true));
        let paused = Arc::new(Mutex::new(false));

        let controller = MotorsSupervisor {
            motors,
            target_positions,
            kp_kd_values,
            running,
            latest_feedback: Arc::new(Mutex::new(HashMap::new())),
            motors_to_zero,
            sleep_duration: Arc::new(Mutex::new(Duration::from_micros(10))),
            paused,
        };

        controller.start_control_thread();

        Ok(controller)
    }

    fn start_control_thread(&self) {
        let motors = Arc::clone(&self.motors);
        let target_positions = Arc::clone(&self.target_positions);
        let kp_kd_values = Arc::clone(&self.kp_kd_values);
        let running = Arc::clone(&self.running);
        let latest_feedback = Arc::clone(&self.latest_feedback);
        let motors_to_zero = Arc::clone(&self.motors_to_zero);
        let sleep_duration = Arc::clone(&self.sleep_duration);
        let paused = Arc::clone(&self.paused);

        thread::spawn(move || {
            let mut motors = motors.lock().unwrap();
            let _ = motors.send_reset();
            let _ = motors.send_can_timeout(100); // If motor doesn't receive a command for 100ms, it will stop.
            let _ = motors.send_start();

            while *running.lock().unwrap() {
                // If paused, just wait 100ms without sending any commands.
                if *paused.lock().unwrap() {
                    std::thread::sleep(Duration::from_millis(100));
                    continue;
                }

                // Read latest feedback from motors.
                {
                    let latest_feedback_from_motors = motors.get_latest_feedback();
                    let mut latest_feedback = latest_feedback.lock().unwrap();
                    *latest_feedback = latest_feedback_from_motors.clone();
                }

                // Send zero torque commands to motors that need to be zeroed.
                {
                    let mut motor_ids_to_zero = motors_to_zero.lock().unwrap();
                    let motor_ids = motor_ids_to_zero.iter().cloned().collect::<Vec<u8>>();
                    if !motor_ids.is_empty() {
                        let _ = motors.send_set_zero(Some(&motor_ids));
                        motor_ids_to_zero.clear();
                    }
                    let torque_commands = HashMap::from_iter(motor_ids.iter().map(|id| (*id, 0.0)));
                    let _ = motors.send_torque_controls(&torque_commands);
                }

                // Send PD commands to motors.
                {
                    /*
                    This code is equivalent, where we run closed-loop torque
                    control, but on the host device rather than on the motor.

                    const TWO_PI: f32 = 2.0 * std::f32::consts::PI;

                    let target_positions = target_positions.lock().unwrap();
                    let kp_kd_values = kp_kd_values.lock().unwrap();

                    let mut torque_commands = HashMap::new();
                    for (&motor_id, &target_position) in target_positions.iter() {
                        if let Ok(feedback) = motors.get_latest_feedback_for(motor_id) {
                            if feedback.position < -TWO_PI || feedback.position > TWO_PI {
                                torque_commands.insert(motor_id, 0.0);
                            } else if let Some(&(kp, kd)) = kp_kd_values.get(&motor_id) {
                                let position_error = target_position - feedback.position;
                                let velocity_error = 0.0 - feedback.velocity; // Assuming we want to stop at the target position

                                let torque = kp * position_error + kd * velocity_error;
                                torque_commands.insert(motor_id, torque);
                            }
                        }
                    }

                    // println!("Torque commands: {:?}", torque_commands);
                    if !torque_commands.is_empty() {
                        let _ = motors.send_torque_controls(&torque_commands);
                    }
                    */

                    let target_positions = target_positions.lock().unwrap();
                    let kp_kd_values = kp_kd_values.lock().unwrap();

                    let pd_sets = HashMap::from_iter(target_positions.iter().map(|(id, pos)| {
                        (*id, (*pos, 0.0, kp_kd_values[id].0, kp_kd_values[id].1))
                    }));
                    if !pd_sets.is_empty() {
                        let _ = motors.send_pd_controls(&pd_sets);
                    }
                }

                {
                    std::thread::sleep(sleep_duration.lock().unwrap().clone());
                }
            }

            let motor_ids: Vec<u8> = motors
                .get_latest_feedback()
                .keys()
                .cloned()
                .collect::<Vec<u8>>();

            let zero_torque_sets: HashMap<u8, f32> =
                HashMap::from_iter(motor_ids.iter().map(|id| (*id, 0.0)));
            let _ = motors.send_torque_controls(&zero_torque_sets);
            let _ = motors.send_reset();
        });
    }

    pub fn set_target_position(&self, motor_id: u8, position: f32) {
        let mut target_positions = self.target_positions.lock().unwrap();
        target_positions.insert(motor_id, position);
    }

    pub fn set_kp_kd(&self, motor_id: u8, kp: f32, kd: f32) {
        let mut kp_kd_values = self.kp_kd_values.lock().unwrap();
        kp_kd_values.insert(motor_id, (kp, kd));
    }

    pub fn set_sleep_duration(&self, sleep_duration: Duration) {
        let mut sleep_duration_to_set = self.sleep_duration.lock().unwrap();
        *sleep_duration_to_set = sleep_duration;
    }

    pub fn add_motor_to_zero(&self, motor_id: u8) {
        let mut motors_to_zero = self.motors_to_zero.lock().unwrap();
        motors_to_zero.insert(motor_id);
    }

    pub fn get_latest_feedback(&self) -> HashMap<u8, MotorFeedback> {
        let latest_feedback = self.latest_feedback.lock().unwrap();
        latest_feedback.clone()
    }

    pub fn toggle_pause(&self) {
        let mut paused = self.paused.lock().unwrap();
        *paused = !*paused;
    }

    pub fn stop(&self) {
        {
            let mut running = self.running.lock().unwrap();
            *running = false;
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

impl Drop for MotorsSupervisor {
    fn drop(&mut self) {
        self.stop();
    }
}

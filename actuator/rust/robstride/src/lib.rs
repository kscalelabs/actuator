use log::{error, info};
use nix;
use serialport::TTYPort;
use std::collections::{HashMap, HashSet};
use std::io::{Read, Write};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration;

#[macro_use]
extern crate lazy_static;

mod port;
use port::init_serial_port;

pub const CAN_ID_MASTER: u8 = 0x00;
pub const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F;
pub const CAN_ID_BROADCAST: u8 = 0xFE;
pub const CAN_ID_DEBUG_UI: u8 = 0xFD;

pub const BAUDRATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B921600;

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
    pub zero_on_init: bool,
    pub can_timeout_command: u16,
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
                zero_on_init: true, // Single encoder motor.
                can_timeout_command: 0x200c,
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
                kd_min: 0.0,
                kd_max: 5.0,
                t_min: -12.0,
                t_max: 12.0,
                zero_on_init: false,
                can_timeout_command: 0x200b,
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
                zero_on_init: false,
                can_timeout_command: 0x200b,
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
                zero_on_init: false,
                can_timeout_command: 0x200b,
            },
        );
        m
    };
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq)]
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

fn pack_bits(values: &[u32], bit_lengths: &[u8]) -> u32 {
    let mut result: u32 = 0;
    let mut current_shift = 0;

    for (&value, &bits) in values.iter().zip(bit_lengths.iter()) {
        let mask = (1 << bits) - 1;
        result |= (value & mask) << current_shift;
        current_shift += bits;
    }

    result
}

fn unpack_bits(value: u32, bit_lengths: &[u8]) -> Vec<u32> {
    let mut result = Vec::new();
    let mut current_value = value;

    for &bits in bit_lengths.iter() {
        let mask = (1 << bits) - 1;
        result.push(current_value & mask);
        current_value >>= bits;
    }

    result
}

fn pack_ex_id(ex_id: &ExId) -> [u8; 4] {
    let addr = (pack_bits(
        &[
            ex_id.id as u32,
            ex_id.data as u32,
            ex_id.mode as u32,
            ex_id.res as u32,
        ],
        &[8, 16, 5, 3],
    ) << 3)
        | 0x00000004;
    addr.to_be_bytes()
}

fn unpack_ex_id(addr: [u8; 4]) -> ExId {
    let addr = u32::from_be_bytes(addr);
    let addr = unpack_bits(addr >> 3, &[8, 16, 5, 3]);
    ExId {
        id: addr[0] as u8,
        data: addr[1] as u16,
        mode: unsafe { std::mem::transmute(addr[2] as u8) },
        res: addr[3] as u8,
    }
}

fn tx_packs(port: &mut TTYPort, packs: &[CanPack], verbose: bool) -> Result<(), std::io::Error> {
    let mut buffer = Vec::new();

    for pack in packs {
        buffer.extend_from_slice(b"AT");
        buffer.extend_from_slice(&pack_ex_id(&pack.ex_id));
        buffer.push(pack.len);
        buffer.extend_from_slice(&pack.data[..pack.len as usize]);
        buffer.extend_from_slice(b"\r\n");
    }

    if buffer.is_empty() {
        return Err(std::io::Error::new(
            std::io::ErrorKind::UnexpectedEof,
            "Empty buffer",
        ));
    }

    if verbose {
        println!(
            "TX: {}",
            buffer
                .iter()
                .map(|b| format!("{:02X}", b))
                .collect::<Vec<String>>()
                .join(" ")
        );
    }

    port.write_all(&buffer)?;
    port.flush()?;

    Ok(())
}

fn rx_unpack(port: &mut TTYPort, len: usize, verbose: bool) -> std::io::Result<Vec<CanPack>> {
    let mut packs = Vec::new();
    while packs.len() < len {
        let mut buffer = [0u8; 17];
        match port.read_exact(&mut buffer) {
            Ok(_) => (),
            Err(e)
                if e.kind() == std::io::ErrorKind::TimedOut
                    || e.kind() == std::io::ErrorKind::UnexpectedEof =>
            {
                break;
            }
            Err(e) => {
                return Err(e);
            }
        }

        if verbose {
            println!(
                "RX: {}",
                buffer
                    .iter()
                    .map(|b| format!("{:02X}", b))
                    .collect::<Vec<String>>()
                    .join(" ")
            );
        }

        if buffer.len() == 17 && buffer[0] == b'A' && buffer[1] == b'T' {
            let ex_id = unpack_ex_id([buffer[2], buffer[3], buffer[4], buffer[5]]);
            let len = buffer[6];

            packs.push(CanPack {
                ex_id,
                len,
                data: buffer[7..(7 + len as usize)].to_vec(),
            });
        } else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Failed to read CAN packet",
            ));
        }
    }

    port.flush()?;

    Ok(packs)
}

fn unpack_raw_feedback(pack: &CanPack) -> MotorFeedbackRaw {
    let can_id = (pack.ex_id.data & 0x00FF) as u8;
    let faults = (pack.ex_id.data & 0x3F00) >> 8;
    let mode = unsafe { std::mem::transmute(((pack.ex_id.data & 0xC000) >> 14) as u8) };

    if pack.ex_id.mode != CanComMode::MotorFeedback {
        return MotorFeedbackRaw {
            can_id,
            pos_int: 0,
            vel_int: 0,
            torque_int: 0,
            mode,
            faults,
        };
    }

    let pos_int = u16::from_be_bytes([pack.data[0], pack.data[1]]);
    let vel_int = u16::from_be_bytes([pack.data[2], pack.data[3]]);
    let torque_int = u16::from_be_bytes([pack.data[4], pack.data[5]]);

    MotorFeedbackRaw {
        can_id,
        pos_int,
        vel_int,
        torque_int,
        mode,
        faults,
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

#[derive(Debug, Clone, Copy)]
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

pub struct Motors {
    port: TTYPort,
    motor_configs: HashMap<u8, &'static MotorConfig>,
    mode: RunMode,
    sleep_time: Duration,
    verbose: bool,
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
            sleep_time: Duration::from_millis(10),
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

        // Reset.
        for &id in &ids_to_zero {
            self.send_reset(id)?;
        }

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

            self.send_command(&pack, true)?;
        }

        // Start.
        for &id in &ids_to_zero {
            self.send_start(id)?;
        }

        Ok(())
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
        tx_packs(&mut self.port, &[pack], self.verbose)?;

        let packs = rx_unpack(&mut self.port, num_packs as usize, self.verbose)?;

        let name = packs
            .iter()
            .flat_map(|pack| pack.data[4..8].iter())
            .map(|&b| b as char)
            .filter(|&c| c != '\0') // Filter out null characters
            .collect::<String>();
        Ok(name)
    }

    fn read_uint16_param(&mut self, motor_id: u8, index: u16) -> Result<u16, std::io::Error> {
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
        tx_packs(&mut self.port, &[pack], self.verbose)?;

        let packs = rx_unpack(&mut self.port, 1, self.verbose)?;
        if packs.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::UnexpectedEof,
                "Failed to receive CAN packet",
            ));
        }
        let pack = packs[0].clone();
        let value = u16::from_le_bytes(pack.data[4..6].try_into().unwrap());
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

    pub fn read_can_timeouts(&mut self) -> Result<HashMap<u8, f32>, std::io::Error> {
        let mut timeouts = HashMap::new();

        for (&id, config) in self.motor_configs.clone().iter() {
            let timeout = self.read_uint16_param(id, config.can_timeout_command)?;
            timeouts.insert(id, timeout as f32 / 20.0);
        }
        Ok(timeouts)
    }

    pub fn send_can_timeout(&mut self, timeout: f32) -> Result<(), std::io::Error> {
        for (&id, config) in self.motor_configs.clone().iter() {
            let cur_timeout = self.read_uint16_param(id, config.can_timeout_command)?;
            let new_timeout = (timeout * 20.0).round().clamp(0.0, 100000.0) as u32;
            if cur_timeout as u32 == new_timeout {
                continue;
            }

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

            pack.data[4..8].copy_from_slice(&new_timeout.to_le_bytes());

            self.send_command(&pack, true)?;
        }

        Ok(())
    }

    fn send_reset(&mut self, id: u8) -> Result<CanPack, std::io::Error> {
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
        let packs: Vec<CanPack> = motor_ids
            .iter()
            .map(|&id| CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::MotorReset,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            })
            .collect();

        self.send_commands(&packs, true, true)?;
        Ok(())
    }

    fn send_start(&mut self, id: u8) -> Result<CanPack, std::io::Error> {
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
        let packs: Vec<CanPack> = motor_ids
            .iter()
            .map(|&id| CanPack {
                ex_id: ExId {
                    id,
                    data: CAN_ID_DEBUG_UI as u16,
                    mode: CanComMode::MotorIn,
                    res: 0,
                },
                len: 8,
                data: vec![0; 8],
            })
            .collect();

        self.send_commands(&packs, true, true)?;
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

pub struct MotorsSupervisor {
    motors: Arc<Mutex<Motors>>,
    target_params: Arc<RwLock<HashMap<u8, MotorControlParams>>>,
    running: Arc<RwLock<bool>>,
    latest_feedback: Arc<RwLock<HashMap<u8, MotorFeedback>>>,
    motors_to_zero: Arc<Mutex<HashSet<u8>>>,
    paused: Arc<RwLock<bool>>,
    restart: Arc<Mutex<bool>>,
    total_commands: Arc<RwLock<u64>>,
    failed_commands: Arc<RwLock<HashMap<u8, u64>>>,
    max_update_rate: Arc<RwLock<f64>>,
    actual_update_rate: Arc<RwLock<f64>>,
    serial: Arc<RwLock<bool>>,
    can_timeout: f32,
}

impl MotorsSupervisor {
    pub fn new(
        port_name: &str,
        motor_infos: &HashMap<u8, MotorType>,
        verbose: bool,
        max_update_rate: f64,
        can_timeout: f32,
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
            .filter(|(_, &config)| config.zero_on_init)
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
            paused: Arc::new(RwLock::new(false)),
            restart: Arc::new(Mutex::new(false)),
            total_commands: Arc::new(RwLock::new(total_commands)),
            failed_commands: Arc::new(RwLock::new(failed_commands)),
            max_update_rate: Arc::new(RwLock::new(max_update_rate)),
            actual_update_rate: Arc::new(RwLock::new(0.0)),
            serial: Arc::new(RwLock::new(true)),
            can_timeout,
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
        let total_commands = Arc::clone(&self.total_commands);
        let failed_commands = Arc::clone(&self.failed_commands);
        let max_update_rate = Arc::clone(&self.max_update_rate);
        let actual_update_rate = Arc::clone(&self.actual_update_rate);
        let serial = Arc::clone(&self.serial);
        let can_timeout = self.can_timeout;

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
            let _ = motors.send_can_timeout(can_timeout);
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
                        let torque_commands = HashMap::from_iter(
                            motor_ids
                                .iter()
                                .map(|id| (*id, MotorControlParams::default())),
                        );
                        let _ = motors.send_motor_controls(&torque_commands, true);
                        let _ = motors.send_set_zeros(Some(&motor_ids));
                        motor_ids_to_zero.clear();
                    }
                }

                {
                    // Send PD commands to motors.
                    let target_params = target_params.read().unwrap();
                    if !target_params.is_empty() {
                        match motors.send_motor_controls(&target_params, *serial.read().unwrap()) {
                            Ok(feedbacks) => {
                                let mut latest_feedback = latest_feedback.write().unwrap();
                                let mut failed_commands = failed_commands.write().unwrap();
                                for &motor_id in target_params.keys() {
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

    pub fn set_params(&self, motor_id: u8, params: MotorControlParams) {
        let mut target_params = self.target_params.write().unwrap();
        target_params.insert(motor_id, params);
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

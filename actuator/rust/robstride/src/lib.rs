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
    pub static ref ROBSTRIDE_CONFIGS: HashMap<&'static str, MotorConfig> = {
        let mut m = HashMap::new();
        m.insert(
            "01",
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
            "03",
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
            "04",
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

#[derive(Copy, Clone, PartialEq)]
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
    pub data: [u8; 8],
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
    println!("Serial port initialized successfully");
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

fn txd_pack(port: &mut Box<dyn SerialPort>, pack: &CanPack) -> Result<(), std::io::Error> {
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

    // println!("tx {:02X?}", buffer);

    port.write_all(&buffer)?;
    port.flush()?;
    Ok(())
}

fn read_bytes(port: &mut Box<dyn SerialPort>) -> Result<MotorFeedbackRaw, std::io::Error> {
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

        let can_id = (ex_id.data & 0x00FF) as u8;
        let faults = (ex_id.data & 0x3F00) >> 8;
        let mode = unsafe { std::mem::transmute(((ex_id.data & 0xC000) >> 14) as u8) };

        let pos_int = u16::from_be_bytes([buffer[7], buffer[8]]);
        let vel_int = u16::from_be_bytes([buffer[9], buffer[10]]);
        let torque_int = u16::from_be_bytes([buffer[11], buffer[12]]);

        Ok(MotorFeedbackRaw {
            can_id,
            pos_int,
            vel_int,
            torque_int,
            mode,
            faults,
        })
    } else {
        Ok(MotorFeedbackRaw::default())
    }
}

pub struct Motor {
    config: &'static MotorConfig,
    id: u8,
}

impl Motor {
    pub fn new(config: &'static MotorConfig, id: u8) -> Self {
        Motor { config, id }
    }
}

pub struct Motors {
    port: Box<dyn SerialPort>,
    motors: HashMap<u8, Motor>,
    latest_feedback: HashMap<u8, MotorFeedback>,
    pending_responses: usize,
}

impl Motors {
    pub fn new(
        port_name: &str,
        motors: HashMap<u8, Motor>,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let port = init_serial_port(port_name)?;
        Ok(Motors {
            port,
            motors,
            latest_feedback: HashMap::new(),
            pending_responses: 0,
        })
    }

    fn send_command(&mut self, pack: &CanPack) -> Result<(), std::io::Error> {
        txd_pack(&mut self.port, pack)?;
        self.pending_responses += 1;
        Ok(())
    }

    pub fn send_set_mode(&mut self, motor_id: u8, mode: RunMode) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7005;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4] = mode as u8;

        self.send_command(&pack)
    }

    pub fn send_reset(&mut self, motor_id: u8) -> Result<(), std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorReset,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.send_command(&pack)
    }

    pub fn send_start(&mut self, motor_id: u8) -> Result<(), std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorIn,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.send_command(&pack)
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
            data: [1, 0, 0, 0, 0, 0, 0, 0], // Set first byte to 1 as per documentation
        };

        self.send_command(&pack)
    }

    pub fn send_set_speed_limit(&mut self, motor_id: u8, speed: f32) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7017;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4..8].copy_from_slice(&speed.to_le_bytes());

        self.send_command(&pack)
    }

    pub fn send_set_location(&mut self, motor_id: u8, location: f32) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: motor_id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7016;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4..8].copy_from_slice(&location.to_le_bytes());

        self.send_command(&pack)
    }

    fn send_motor_control(
        &mut self,
        motor_id: u8,
        pos_set: f32,
        vel_set: f32,
        kp_set: f32,
        kd_set: f32,
        torque_set: f32,
    ) -> Result<(), std::io::Error> {
        if let Some(motor) = self.motors.get(&motor_id) {
            let mut pack = CanPack {
                ex_id: ExId {
                    id: motor_id as u8,
                    data: 0,
                    mode: CanComMode::MotorCtrl,
                    res: 0,
                },
                len: 8,
                data: [0; 8],
            };

            let pos_int_set = float_to_uint(pos_set, motor.config.p_min, motor.config.p_max, 16);
            let vel_int_set = float_to_uint(vel_set, motor.config.v_min, motor.config.v_max, 16);
            let kp_int_set = float_to_uint(kp_set, motor.config.kp_min, motor.config.kp_max, 16);
            let kd_int_set = float_to_uint(kd_set, motor.config.kd_min, motor.config.kd_max, 16);
            let torque_int_set = float_to_uint(torque_set, motor.config.t_min, motor.config.t_max, 16);

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
            Err(std::io::Error::new(std::io::ErrorKind::NotFound, "Motor not found"))
        }
    }

    pub fn send_position_control(&mut self, motor_id: u8, pos_set: f32, kp_set: f32, kd_set: f32) -> Result<(), std::io::Error> {
        self.send_motor_control(motor_id, pos_set, 0.0, kp_set, kd_set, 0.0)
    }

    pub fn send_torque_control(&mut self, motor_id: u8, torque_set: f32) -> Result<(), std::io::Error> {
        self.send_motor_control(motor_id, 0.0, 0.0, 0.0, 0.0, torque_set)
    }

    pub fn read_all_pending_responses(&mut self) -> Result<HashMap<u8, MotorFeedback>, std::io::Error> {
        while self.pending_responses > 0 {
            match read_bytes(&mut self.port) {
                Ok(raw_feedback) => {
                    if let Some(motor) = self.motors.get(&raw_feedback.can_id) {
                        let position = uint_to_float(raw_feedback.pos_int, motor.config.p_min, motor.config.p_max, 16);
                        let velocity = uint_to_float(raw_feedback.vel_int, motor.config.v_min, motor.config.v_max, 16);
                        let torque = uint_to_float(raw_feedback.torque_int, motor.config.t_min, motor.config.t_max, 16);

                        let feedback = MotorFeedback {
                            can_id: raw_feedback.can_id,
                            position,
                            velocity,
                            torque,
                            mode: raw_feedback.mode,
                            faults: raw_feedback.faults,
                        };

                        self.latest_feedback.insert(motor.id, feedback);
                    }
                    self.pending_responses -= 1;
                }
                Err(e) => {
                    self.pending_responses -= 1;
                    eprintln!("Error reading response: {:?}", e);
                }
            }
        }
        Ok(self.latest_feedback.clone())
    }

    pub fn get_latest_feedback(&self, motor_id: u8) -> Option<&MotorFeedback> {
        self.latest_feedback.get(&motor_id)
    }
}

fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let offset = x_min;
    ((x - offset) * ((1 << bits) - 1) as f32 / span) as u16
}

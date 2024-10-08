use serialport::SerialPort;
use std::collections::HashMap;
use std::io::{Read, Write};
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

#[derive(Debug, Default)]
pub struct MotorFeedback {
    pub can_id: u16,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
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

fn read_bytes(
    port: &mut Box<dyn SerialPort>,
    config: &MotorConfig,
) -> Result<MotorFeedback, std::io::Error> {
    let mut buffer = [0u8; 17];
    let bytes_read = port.read(&mut buffer)?;

    // println!("rx {:02X?}", &buffer[..bytes_read]);

    if bytes_read == 17 && buffer[0] == b'A' && buffer[1] == b'T' {
        let addr = u32::from_be_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]) >> 3;
        let ex_id = ExId {
            id: (addr & 0xFF) as u8,
            data: ((addr >> 8) & 0xFFFF) as u16,
            mode: unsafe { std::mem::transmute((addr >> 24) as u8) },
            res: 0,
        };

        let can_id = ex_id.data & 0x00FF;
        let faults = (ex_id.data & 0x3F00) >> 8;
        let mode = unsafe { std::mem::transmute(((ex_id.data & 0xC000) >> 14) as u8) };

        let pos_int_get = u16::from_be_bytes([buffer[7], buffer[8]]);
        let vel_int_get = u16::from_be_bytes([buffer[9], buffer[10]]);
        let torque_int_get = u16::from_be_bytes([buffer[11], buffer[12]]);

        let position = uint_to_float(pos_int_get, config.p_min, config.p_max, 16);
        let velocity = uint_to_float(vel_int_get, config.v_min, config.v_max, 16);
        let torque = uint_to_float(torque_int_get, config.t_min, config.t_max, 16);

        let feedback = MotorFeedback {
            can_id,
            position,
            velocity,
            torque,
            mode,
            faults,
        };

        // println!("Parsed data:");
        // println!("  Motor ID: {}", feedback.can_id);
        // println!("  Position: {}", feedback.position);
        // println!("  Velocity: {}", feedback.velocity);
        // println!("  Torque: {}", feedback.torque);
        // println!("  Mode: {:?}", feedback.mode);
        // println!("  Faults: {:?}", feedback.faults);

        Ok(feedback)
    } else {
        Ok(MotorFeedback {
            can_id: 0,
            position: 0.0,
            velocity: 0.0,
            torque: 0.0,
            mode: MotorMode::Reset,
            faults: 0,
        })
    }
}

pub struct Motor {
    port: Box<dyn SerialPort>,
    config: &'static MotorConfig,
    id: u8,
    pending_responses: usize,
}

impl Motor {
    pub fn new(
        tty_port: &str,
        config: &'static MotorConfig,
        id: u8,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let port = init_serial_port(tty_port)?;
        Ok(Motor {
            port,
            config,
            id,
            pending_responses: 0,
        })
    }

    fn send_command(&mut self, pack: &CanPack) -> Result<(), std::io::Error> {
        txd_pack(&mut self.port, pack)?;
        self.pending_responses += 1;
        Ok(())
    }

    pub fn send_set_mode(&mut self, run_mode: RunMode) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: self.id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::SdoWrite,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let index: u16 = 0x7005;
        pack.data[..2].copy_from_slice(&index.to_le_bytes());
        pack.data[4] = run_mode as u8;

        self.send_command(&pack)?;
        Ok(())
    }

    pub fn send_reset(&mut self) -> Result<(), std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id: self.id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorReset,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.send_command(&pack)
    }

    pub fn send_start(&mut self) -> Result<(), std::io::Error> {
        let pack = CanPack {
            ex_id: ExId {
                id: self.id,
                data: CAN_ID_DEBUG_UI as u16,
                mode: CanComMode::MotorIn,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        self.send_command(&pack)
    }

    pub fn send_set_speed_limit(&mut self, speed: f32) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: self.id,
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

    pub fn send_set_location(&mut self, location: f32) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: self.id,
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
        pos_set: f32,
        vel_set: f32,
        kp_set: f32,
        kd_set: f32,
        torque_set: f32,
    ) -> Result<(), std::io::Error> {
        let mut pack = CanPack {
            ex_id: ExId {
                id: self.id,
                data: 0,
                mode: CanComMode::MotorCtrl,
                res: 0,
            },
            len: 8,
            data: [0; 8],
        };

        let pos_int_set = float_to_uint(pos_set, self.config.p_min, self.config.p_max, 16);
        let vel_int_set = float_to_uint(vel_set, self.config.v_min, self.config.v_max, 16);
        let kp_int_set = float_to_uint(kp_set, self.config.kp_min, self.config.kp_max, 16);
        let kd_int_set = float_to_uint(kd_set, self.config.kd_min, self.config.kd_max, 16);
        let torque_int_set = float_to_uint(torque_set, self.config.t_min, self.config.t_max, 16);

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
    }

    pub fn read_all_pending_responses(&mut self) -> Result<Vec<MotorFeedback>, std::io::Error> {
        let mut feedbacks = Vec::new();
        while self.pending_responses > 0 {
            match read_bytes(&mut self.port, self.config) {
                Ok(feedback) => {
                    feedbacks.push(feedback);
                    self.pending_responses -= 1;
                }
                Err(e) => {
                    // If there's an error, we'll still decrement pending_responses
                    // to avoid getting stuck in an infinite loop
                    self.pending_responses -= 1;
                    eprintln!("Error reading response: {:?}", e);
                }
            }
        }
        Ok(feedbacks)
    }

    // Note that you need to set the RunMode to MitMode before calling this
    pub fn send_position_control(&mut self, pos_set: f32, kp_set: f32) -> Result<(), std::io::Error> {
        self.send_motor_control(pos_set, 0.0, kp_set, 0.0, 0.0)
    }

    // Note that you need to set the RunMode to MitMode before calling this
    pub fn send_torque_control(&mut self, torque_set: f32) -> Result<(), std::io::Error> {
        self.send_motor_control(0.0, 0.0, 0.0, 0.0, torque_set)
    }
}

// Add this helper function outside of the Motor impl block
fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let offset = x_min;
    ((x - offset) * ((1 << bits) - 1) as f32 / span) as u16
}

pub struct Motors {
    motors: Vec<Motor>,
    current_mode: RunMode,
}

impl Motors {
    pub fn new(
        tty_port: &str,
        motor_configs: &[(&'static MotorConfig, u8)],
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let motors = motor_configs
            .iter()
            .map(|(config, id)| Motor::new(tty_port, config, *id))
            .collect::<Result<Vec<Motor>, Box<dyn std::error::Error>>>()?;

        Ok(Motors {
            motors,
            current_mode: RunMode::UnsetMode,
        })
    }

    pub fn send_set_mode(&mut self, run_mode: RunMode) -> Result<(), std::io::Error> {
        if self.current_mode == run_mode {
            return Ok(());
        }
        for motor in &mut self.motors {
            motor.send_set_mode(run_mode)?;
        }
        self.current_mode = run_mode;
        Ok(())
    }

    pub fn send_set_locations(
        &mut self,
        locations: &HashMap<u8, f32>,
    ) -> Result<(), std::io::Error> {
        self.send_set_mode(RunMode::PositionMode)?;

        for motor in &mut self.motors {
            if let Some(&location) = locations.get(&motor.id) {
                motor.send_set_location(location)?;
            }
        }
        Ok(())
    }

    pub fn send_set_speed_limits(
        &mut self,
        speed_limits: &HashMap<u8, f32>,
    ) -> Result<(), std::io::Error> {
        for motor in &mut self.motors {
            if let Some(&speed_limit) = speed_limits.get(&motor.id) {
                motor.send_set_speed_limit(speed_limit)?;
            }
        }
        // Sleep
        thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    pub fn read_all_pending_responses(
        &mut self,
    ) -> Result<HashMap<u16, MotorFeedback>, std::io::Error> {
        let mut feedbacks = HashMap::new();
        for motor in &mut self.motors {
            for feedback in motor.read_all_pending_responses()? {
                feedbacks.insert(feedback.can_id, feedback);
            }
        }
        Ok(feedbacks)
    }

    // fn send_position_control(&mut self, pos_set: f32, kp_set: f32) -> Result<(), std::io::Error> {
    //     self.send_motor_control(pos_set, 0.0, kp_set, 0.0, 0.0)
    // }

    // fn send_torque_control(&mut self, torque_set: f32) -> Result<(), std::io::Error> {
    //     self.send_motor_control(0.0, 0.0, 0.0, 0.0, torque_set)
    // }
}

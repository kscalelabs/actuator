use serialport::SerialPort;
use std::io::{Read, Write};
use std::time::Duration;

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

pub const ROBSTRIDE01_CONFIG: MotorConfig = MotorConfig {
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
};

pub const ROBSTRIDE03_CONFIG: MotorConfig = MotorConfig {
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
};

pub const ROBSTRIDE04_CONFIG: MotorConfig = MotorConfig {
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
};

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
#[derive(Copy, Clone, Debug)]
pub enum MotorMode {
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

pub enum RunMode {
    MitMode = 0,
    PositionMode,
    SpeedMode,
    CurrentMode,
    ToZeroMode,
    CspPositionMode,
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

pub struct MotorFeedback {
    pub can_id: u16,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mode: MotorMode,
    pub faults: u16,
}

pub struct Motor {
    port: Box<dyn SerialPort>,
    config: MotorConfig,
    id: u8,
}

impl Motor {
    pub fn new(
        tty_port: &str,
        config: MotorConfig,
        id: u8,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let port = init_serial_port(tty_port)?;
        Ok(Motor { port, config, id })
    }

    fn txd_pack(&mut self, pack: &CanPack) -> Result<(), std::io::Error> {
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

        println!("tx {:02X?}", buffer);

        self.port.write_all(&buffer)?;
        self.port.flush()?;
        Ok(())
    }

    fn read_bytes(&mut self) -> Result<MotorFeedback, std::io::Error> {
        let mut buffer = [0u8; 17];
        let bytes_read = self.port.read(&mut buffer)?;

        println!("rx {:02X?}", &buffer[..bytes_read]);

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

            let position = uint_to_float(pos_int_get, self.config.p_min, self.config.p_max, 16);
            let velocity = uint_to_float(vel_int_get, self.config.v_min, self.config.v_max, 16);
            let torque = uint_to_float(torque_int_get, self.config.t_min, self.config.t_max, 16);

            let feedback = MotorFeedback {
                can_id,
                position,
                velocity,
                torque,
                mode,
                faults,
            };

            println!("Parsed data:");
            println!("  Motor ID: {}", feedback.can_id);
            println!("  Position: {}", feedback.position);
            println!("  Velocity: {}", feedback.velocity);
            println!("  Torque: {}", feedback.torque);
            println!("  Mode: {:?}", feedback.mode);
            println!("  Faults: {:?}", feedback.faults);

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

    pub fn send_set_mode(&mut self, run_mode: RunMode) -> Result<MotorFeedback, std::io::Error> {
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

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_reset(&mut self) -> Result<MotorFeedback, std::io::Error> {
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

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_start(&mut self) -> Result<MotorFeedback, std::io::Error> {
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

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_set_speed_limit(&mut self, speed: f32) -> Result<MotorFeedback, std::io::Error> {
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

        self.txd_pack(&pack)?;
        self.read_bytes()
    }

    pub fn send_set_location(&mut self, location: f32) -> Result<MotorFeedback, std::io::Error> {
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

        self.txd_pack(&pack)?;
        self.read_bytes()
    }
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

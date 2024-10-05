use serialport::SerialPort;
use std::io::{Read, Write};
use std::thread;
use std::time::Duration;

const CAN_ID_MASTER: u8 = 0x00;
const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F;
const CAN_ID_BROADCAST: u8 = 0xFE;
const CAN_ID_DEBUG_UI: u8 = 0xFD;

const BAUDRATE: u32 = 921600;
const TTY_PORT: &str = "/dev/ttyUSB0";

const P_MIN: f32 = -12.5;
const P_MAX: f32 = 12.5;
const V_MIN: f32 = -44.0;
const V_MAX: f32 = 44.0;
const KP_MIN: f32 = 0.0;
const KP_MAX: f32 = 500.0;
const KD_MIN: f32 = 0.0;
const KD_MAX: f32 = 5.0;
const T_MIN: f32 = -12.0;
const T_MAX: f32 = 12.0;

#[repr(u8)]
#[derive(Copy, Clone)]
enum CanComMode {
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
enum MotorMode {
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

enum RunMode {
    MitMode = 0,
    PositionMode,
    SpeedMode,
    CurrentMode,
    ToZeroMode,
    CspPositionMode,
}

struct ExId {
    id: u8,
    data: u16,
    mode: CanComMode,
    res: u8,
}

struct CanPack {
    ex_id: ExId,
    len: u8,
    data: [u8; 8],
}

struct MotorFeedback {
    can_id: u16,
    position: f32,
    velocity: f32,
    torque: f32,
    mode: MotorMode,
    faults: u16,
    is_set: bool,
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

fn txd_pack(port: &mut Box<dyn SerialPort>, pack: &CanPack) -> Result<(), std::io::Error> {
    let mut buffer = Vec::new();
    buffer.extend_from_slice(b"AT");

    let addr = (pack.ex_id.id as u32) << 24
        | (pack.ex_id.data as u32) << 8
        | (pack.ex_id.mode as u32) << 3
        | 0x00000004;

    buffer.extend_from_slice(&addr.to_be_bytes());
    buffer.push(pack.len);
    buffer.extend_from_slice(&pack.data[..pack.len as usize]);
    buffer.extend_from_slice(b"\r\n");

    println!("tx {:02X?}", buffer);

    port.write_all(&buffer)?;
    port.flush()?;
    Ok(())
}

fn uint_to_float(x_int: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span = x_max - x_min;
    let offset = x_min;
    (x_int as f32) * span / ((1 << bits) - 1) as f32 + offset
}

fn read_bytes(port: &mut Box<dyn SerialPort>) -> Result<MotorFeedback, std::io::Error> {
    let mut buffer = [0u8; 17];
    let bytes_read = port.read(&mut buffer)?;

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

        let position = uint_to_float(pos_int_get, P_MIN, P_MAX, 16);
        let velocity = uint_to_float(vel_int_get, V_MIN, V_MAX, 16);
        let torque = uint_to_float(torque_int_get, T_MIN, T_MAX, 16);

        let feedback = MotorFeedback {
            can_id,
            position,
            velocity,
            torque,
            mode,
            faults,
            is_set: true,
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
            is_set: false,
        })
    }
}

fn send_set_mode(
    port: &mut Box<dyn SerialPort>,
    run_mode: RunMode,
    id: u8,
) -> Result<MotorFeedback, std::io::Error> {
    let mut pack = CanPack {
        ex_id: ExId {
            id,
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

    txd_pack(port, &pack)?;
    read_bytes(port)
}

fn send_reset(port: &mut Box<dyn SerialPort>, id: u8) -> Result<MotorFeedback, std::io::Error> {
    let pack = CanPack {
        ex_id: ExId {
            id,
            data: CAN_ID_DEBUG_UI as u16,
            mode: CanComMode::MotorReset,
            res: 0,
        },
        len: 8,
        data: [0; 8],
    };

    txd_pack(port, &pack)?;
    read_bytes(port)
}

fn send_start(port: &mut Box<dyn SerialPort>, id: u8) -> Result<MotorFeedback, std::io::Error> {
    let pack = CanPack {
        ex_id: ExId {
            id,
            data: CAN_ID_DEBUG_UI as u16,
            mode: CanComMode::MotorIn,
            res: 0,
        },
        len: 8,
        data: [0; 8],
    };

    txd_pack(port, &pack)?;
    read_bytes(port)
}

fn send_set_speed_limit(
    port: &mut Box<dyn SerialPort>,
    id: u8,
    speed: f32,
) -> Result<MotorFeedback, std::io::Error> {
    let mut pack = CanPack {
        ex_id: ExId {
            id,
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

    txd_pack(port, &pack)?;
    read_bytes(port)
}

fn send_set_location(
    port: &mut Box<dyn SerialPort>,
    id: u8,
    location: f32,
) -> Result<MotorFeedback, std::io::Error> {
    let mut pack = CanPack {
        ex_id: ExId {
            id,
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

    txd_pack(port, &pack)?;
    read_bytes(port)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting program");

    let mut port = init_serial_port(TTY_PORT)?;
    let id = 2;

    // Set mode to position mode
    let _ = send_set_mode(&mut port, RunMode::PositionMode, id)?;
    thread::sleep(Duration::from_millis(50));

    send_reset(&mut port, id)?;

    // Start the motor
    let _ = send_start(&mut port, id)?;
    thread::sleep(Duration::from_millis(50));

    // Set speed limit
    let _ = send_set_speed_limit(&mut port, id, 5.0)?;
    thread::sleep(Duration::from_millis(50));

    for i in 0..10 {
        // Set location
        let _ = send_set_location(&mut port, id, std::f32::consts::PI * i as f32 / 2.0)?;
        thread::sleep(Duration::from_secs(1));
    }

    let _ = send_set_speed_limit(&mut port, id, 10.0)?;
    let _ = send_set_location(&mut port, id, 0.0)?;
    thread::sleep(Duration::from_secs(2));

    // Reset the motor after the loop
    let _ = send_reset(&mut port, id)?;

    println!("Program finished");
    Ok(())
}

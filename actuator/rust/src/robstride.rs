// firmware/rust/src/lib.rs
use pyo3::prelude::*;
use serialport::SerialPort;
use std::time::Duration;

const MAX_PORTS: usize = 10;

const CAN_ID_MASTER: u8 = 0x00;
const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F;
const CAN_ID_BROADCAST: u8 = 0xFE;
const CAN_ID_DEBUG_UI: u8 = 0xFD;

const BAUDRATE: u32 = 921600;
const DEFAULT_TTY_PORT: &str = "/dev/ttyCH341USB0";

const P_MIN: f32 = -1.0;
const P_MAX: f32 = 1.0;
const V_MIN: f32 = -1.0;
const V_MAX: f32 = 1.0;
const KP_MIN: f32 = 0.0;
const KP_MAX: f32 = 500.0;
const KD_MIN: f32 = 0.0;
const KD_MAX: f32 = 5.0;
const T_MIN: f32 = -18.0;
const T_MAX: f32 = 18.0;

#[repr(u8)]
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
enum MotorMode {
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

struct MotorStatus {
    under_volt_fault: bool,
    over_cur_fault: bool,
    over_temp_fault: bool,
    encoder_fault: bool,
    hall_fault: bool,
    no_cali_fault: bool,
    mode: MotorMode,
}

#[derive(Default)]
struct ExId(u32);

impl ExId {
    fn new(id: u8, data: u16, mode: u8, res: u8) -> ExId {
        let mut value: u32 = 0;
        value |= (res as u32 & 0x07) << 29;
        value |= (mode as u32 & 0x1F) << 24;
        value |= (data as u32 & 0xFFFF) << 8;
        value |= id as u32 & 0xFF;
        ExId(value)
    }

    fn to_addr(&self) -> u32 {
        let exid_u32 = self.0;
        (exid_u32 << 3) | 0x00000004
    }
}

#[derive(Default)]
struct CanPack {
    ex_id: ExId,
    len: u8,
    data: [u8; 8],
}

fn float_to_uint(value: f32, min: f32, max: f32, bits: u32) -> u32 {
    let span = max - min;
    let offset = value - min;
    ((offset / span) * ((1 << bits) - 1) as f32).round() as u32
}

fn txd_pack(serial_port: &mut Box<dyn SerialPort>, pack: &CanPack) {
    println!("Sending data");

    let mut len = pack.len;
    let mut data = pack.data;

    if len < 1 {
        len = 1;
        data[0] = 0;
    } else if len > 8 {
        len = 8;
    }

    let addr = pack.ex_id.to_addr();

    let mut packet = Vec::with_capacity(9 + len as usize);
    packet.push(b'A');
    packet.push(b'T');

    packet.push(((addr & 0xFF000000) >> 24) as u8);
    packet.push(((addr & 0x00FF0000) >> 16) as u8);
    packet.push(((addr & 0x0000FF00) >> 8) as u8);
    packet.push((addr & 0x000000FF) as u8);

    packet.push(len);

    for i in 0..len as usize {
        packet.push(data[i]);
    }

    packet.push(b'\r');
    packet.push(b'\n');

    // Print the full packet
    print!("Full packet (hex): ");
    for byte in &packet {
        print!("{:02X} ", byte);
    }
    println!();

    // Write the data
    match serial_port.write(&packet) {
        Ok(bytes_written) => {
            if bytes_written != packet.len() {
                eprintln!(
                    "Warning: Not all bytes written. Expected {}, wrote {}",
                    packet.len(),
                    bytes_written
                );
            } else {
                println!("Data sent successfully");
            }
        }
        Err(e) => {
            eprintln!("Error writing to serial port: {}", e);
        }
    }

    // Flush the output buffer
    if let Err(e) = serial_port.flush() {
        eprintln!("Error flushing serial port: {}", e);
    }
}

fn init_serial_port(device: &str) -> Result<Box<dyn SerialPort>, serialport::Error> {
    println!("Initializing serial port: {}", device);
    let port = serialport::new(device, BAUDRATE)
        .timeout(Duration::from_secs(1))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        .open();

    match port {
        Ok(port) => {
            println!("Serial port initialized successfully");
            Ok(port)
        }
        Err(e) => {
            eprintln!("Error opening serial port: {}", e);
            Err(e)
        }
    }
}

fn send_reset(serial_port: &mut Box<dyn SerialPort>, id: u8) {
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::MotorReset as u8, 0),
        len: 8,
        data: [0; 8],
    };
    txd_pack(serial_port, &pack);
}

fn send_start(serial_port: &mut Box<dyn SerialPort>, id: u8) {
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::MotorIn as u8, 0),
        len: 8,
        data: [0; 8],
    };
    txd_pack(serial_port, &pack);
}

fn send_set_mode(serial_port: &mut Box<dyn SerialPort>, id: u8, mode: u8) -> Result<(), Box<dyn std::error::Error>> {
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::SdoWrite as u8, 0),
        len: 8,
        data: [0x05, 0x70, 0, 0, mode, 0, 0, 0],
    };
    txd_pack(serial_port, &pack);
    Ok(())
}

fn send_set_speed_limit(serial_port: &mut Box<dyn SerialPort>, id: u8, speed: f32) -> Result<(), Box<dyn std::error::Error>> {
    let speed_bytes = speed.to_le_bytes();
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::SdoWrite as u8, 0),
        len: 8,
        data: [0x17, 0x70, 0, 0, speed_bytes[0], speed_bytes[1], speed_bytes[2], speed_bytes[3]],
    };
    txd_pack(serial_port, &pack);
    Ok(())
}

fn send_set_location(serial_port: &mut Box<dyn SerialPort>, id: u8, location: f32) -> Result<(), Box<dyn std::error::Error>> {
    let location_bytes = location.to_le_bytes();
    let pack = CanPack {
        ex_id: ExId::new(id, CAN_ID_DEBUG_UI as u16, CanComMode::SdoWrite as u8, 0),
        len: 8,
        data: [0x16, 0x70, 0, 0, location_bytes[0], location_bytes[1], location_bytes[2], location_bytes[3]],
    };
    txd_pack(serial_port, &pack);
    Ok(())
}

fn send_motor_control(serial_port: &mut Box<dyn SerialPort>, id: u8, pos_set: f32, vel_set: f32, kp_set: f32, kd_set: f32, torque_set: f32) -> Result<(), Box<dyn std::error::Error>> {
    let pos_int_set = float_to_uint(pos_set, P_MIN, P_MAX, 16) as u16;
    let vel_int_set = float_to_uint(vel_set, V_MIN, V_MAX, 16) as u16;
    let kp_int_set = float_to_uint(kp_set, KP_MIN, KP_MAX, 16) as u16;
    let kd_int_set = float_to_uint(kd_set, KD_MIN, KD_MAX, 16) as u16;
    let torque_int_set = float_to_uint(torque_set, T_MIN, T_MAX, 16) as u16;

    let pack = CanPack {
        ex_id: ExId::new(id, torque_int_set, CanComMode::MotorCtrl as u8, 0),
        len: 8,
        data: [
            (pos_int_set >> 8) as u8,
            (pos_int_set & 0xFF) as u8,
            (vel_int_set >> 8) as u8,
            (vel_int_set & 0xFF) as u8,
            (kp_int_set >> 8) as u8,
            (kp_int_set & 0xFF) as u8,
            (kd_int_set >> 8) as u8,
            (kd_int_set & 0xFF) as u8,
        ],
    };
    txd_pack(serial_port, &pack);
    Ok(())
}

#[pyfunction(signature = (device_id, port=None))]
fn py_send_start(device_id: u8, port: Option<String>) -> PyResult<()> {
    let port = port.unwrap_or_else(|| DEFAULT_TTY_PORT.to_string());
    let serial_port = init_serial_port(&port).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    let mut serial_port = serial_port;
    send_start(&mut serial_port, device_id);
    Ok(())
}

#[pyfunction(signature = (device_id, mode, port=None))]
fn py_send_set_mode(device_id: u8, mode: u8, port: Option<String>) -> PyResult<()> {
    let port = port.unwrap_or_else(|| DEFAULT_TTY_PORT.to_string());
    let serial_port = init_serial_port(&port).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    let mut serial_port = serial_port;
    send_set_mode(&mut serial_port, device_id, mode).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    Ok(())
}

#[pyfunction(signature = (device_id, speed, port=None))]
fn py_send_set_speed_limit(device_id: u8, speed: f32, port: Option<String>) -> PyResult<()> {
    let port = port.unwrap_or_else(|| DEFAULT_TTY_PORT.to_string());
    let serial_port = init_serial_port(&port).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    let mut serial_port = serial_port;
    send_set_speed_limit(&mut serial_port, device_id, speed).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    Ok(())
}

#[pyfunction(signature = (device_id, location, port=None))]
fn py_send_set_location(device_id: u8, location: f32, port: Option<String>) -> PyResult<()> {
    let port = port.unwrap_or_else(|| DEFAULT_TTY_PORT.to_string());
    let serial_port = init_serial_port(&port).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    let mut serial_port = serial_port;
    send_set_location(&mut serial_port, device_id, location).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    Ok(())
}

#[pyfunction(signature = (device_id, pos_set, vel_set, kp_set, kd_set, torque_set, port=None))]
fn py_send_motor_control(device_id: u8, pos_set: f32, vel_set: f32, kp_set: f32, kd_set: f32, torque_set: f32, port: Option<String>) -> PyResult<()> {
    let port = port.unwrap_or_else(|| DEFAULT_TTY_PORT.to_string());
    let serial_port = init_serial_port(&port).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    let mut serial_port = serial_port;
    send_motor_control(&mut serial_port, device_id, pos_set, vel_set, kp_set, kd_set, torque_set).map_err(|e| pyo3::exceptions::PyOSError::new_err(e.to_string()))?;
    Ok(())
}

#[pymodule]
fn robstride(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(py_send_start, m)?)?;
    m.add_function(wrap_pyfunction!(py_send_set_mode, m)?)?;
    m.add_function(wrap_pyfunction!(py_send_set_speed_limit, m)?)?;
    m.add_function(wrap_pyfunction!(py_send_set_location, m)?)?;
    m.add_function(wrap_pyfunction!(py_send_motor_control, m)?)?;
    Ok(())
}

use crate::types::MotorMode;
use serde::{Deserialize, Serialize};
use serialport::TTYPort;
use std::io::{Read, Write};

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExId {
    pub id: u8,
    pub data: u16,
    pub mode: CanComMode,
    pub res: u8,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CanPack {
    pub ex_id: ExId,
    pub len: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct MotorFeedbackRaw {
    pub can_id: u8,
    pub pos_int: u16,
    pub vel_int: u16,
    pub torque_int: u16,
    pub mode: MotorMode,
    pub faults: u16,
}

pub fn uint_to_float(x_int: u16, x_min: f32, x_max: f32, bits: u8) -> f32 {
    let span = x_max - x_min;
    let offset = x_min;
    (x_int as f32) * span / ((1 << bits) - 1) as f32 + offset
}

pub fn float_to_uint(x: f32, x_min: f32, x_max: f32, bits: u8) -> u16 {
    let span = x_max - x_min;
    let offset = x_min;
    ((x - offset) * ((1 << bits) - 1) as f32 / span) as u16
}

pub fn pack_bits(values: &[u32], bit_lengths: &[u8]) -> u32 {
    let mut result: u32 = 0;
    let mut current_shift = 0;

    for (&value, &bits) in values.iter().zip(bit_lengths.iter()) {
        let mask = (1 << bits) - 1;
        result |= (value & mask) << current_shift;
        current_shift += bits;
    }

    result
}

pub fn unpack_bits(value: u32, bit_lengths: &[u8]) -> Vec<u32> {
    let mut result = Vec::new();
    let mut current_value = value;

    for &bits in bit_lengths.iter() {
        let mask = (1 << bits) - 1;
        result.push(current_value & mask);
        current_value >>= bits;
    }

    result
}

pub fn pack_ex_id(ex_id: &ExId) -> [u8; 4] {
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

pub fn unpack_ex_id(addr: [u8; 4]) -> ExId {
    let addr = u32::from_be_bytes(addr);
    let addr = unpack_bits(addr >> 3, &[8, 16, 5, 3]);
    ExId {
        id: addr[0] as u8,
        data: addr[1] as u16,
        mode: unsafe { std::mem::transmute(addr[2] as u8) },
        res: addr[3] as u8,
    }
}

pub fn unpack_raw_feedback(pack: &CanPack) -> MotorFeedbackRaw {
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

pub fn tx_packs(
    port: &mut TTYPort,
    packs: &[CanPack],
    verbose: bool,
) -> Result<(), std::io::Error> {
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

pub fn rx_unpack(port: &mut TTYPort, len: usize, verbose: bool) -> std::io::Result<Vec<CanPack>> {
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

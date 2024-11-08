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

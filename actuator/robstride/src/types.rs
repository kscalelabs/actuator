use serde::{Deserialize, Serialize};

#[repr(u8)]
#[derive(Copy, Clone, Debug, Default, Serialize, Deserialize)]
pub enum MotorMode {
    #[default]
    Reset = 0,
    Cali,
    Motor,
    Brake,
}

#[derive(Debug, Copy, Clone, PartialEq, Serialize, Deserialize)]
pub enum RunMode {
    UnsetMode = -1,
    MitMode = 0,
    PositionMode = 1,
    SpeedMode = 2,
    CurrentMode = 3,
    ToZeroMode = 4,
    CspPositionMode = 5,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MotorType {
    Type00,
    Type01,
    Type02,
    Type03,
    Type04,
}

pub fn motor_mode_from_str(s: &str) -> Result<MotorMode, std::io::Error> {
    match s {
        "Reset" => Ok(MotorMode::Reset),
        "Cali" => Ok(MotorMode::Cali),
        "Motor" => Ok(MotorMode::Motor),
        "Brake" => Ok(MotorMode::Brake),
        _ => Err(std::io::Error::new(
            std::io::ErrorKind::InvalidInput,
            "Invalid motor mode",
        )),
    }
}

pub fn motor_type_from_str(s: &str) -> Result<MotorType, std::io::Error> {
    match s {
        "00" => Ok(MotorType::Type00),
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

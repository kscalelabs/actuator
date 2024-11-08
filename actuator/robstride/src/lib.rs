mod can;
mod config;
mod motor;
mod supervisor;
mod types;

pub use can::{CanComMode, CanPack, ExId};
pub use config::{MotorConfig, ROBSTRIDE_CONFIGS};
pub use motor::{Motor, MotorControlParams, MotorFeedback, MotorFeedbackRaw, Motors};
pub use supervisor::MotorsSupervisor;
pub use types::{MotorMode, MotorType, RunMode};

pub const CAN_ID_MASTER: u8 = 0x00;
pub const CAN_ID_MOTOR_DEFAULT: u8 = 0x7F;
pub const CAN_ID_BROADCAST: u8 = 0xFE;
pub const CAN_ID_DEBUG_UI: u8 = 0xFD;

#[cfg(target_os = "linux")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B921600;

#[cfg(target_os = "macos")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B115200;

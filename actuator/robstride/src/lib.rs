mod can;
mod config;
mod motor;
mod port;
mod supervisor;
mod types;

pub use can::{CanComMode, CanPack, ExId};
pub use config::{MotorConfig, ROBSTRIDE_CONFIGS};
pub use motor::{MotorControlParams, MotorFeedback, Motors};
pub use supervisor::MotorsSupervisor;
pub use types::{motor_mode_from_str, motor_type_from_str, MotorMode, MotorType, RunMode};

#[cfg(target_os = "linux")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B921600;

#[cfg(target_os = "macos")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B115200;

// Defines error types for the actuator crate.

use thiserror::Error;

#[derive(Error, Debug)]
pub enum ActuatorError {
    #[error("Failed to connect to the actuator server")]
    ConnectionError,
}

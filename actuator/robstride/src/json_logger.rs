//! High-performance JSON logging for actuator commands and feedback
use serde::{Deserialize, Serialize};
use std::io::Write;
use eyre::Result;
use std::path::Path;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::io::AsyncWriteExt;
use tokio::sync::mpsc;
use tokio::sync::Mutex;
use tokio::fs::OpenOptions;
use tokio::time::{Duration, Instant};
use tracing::{debug, error, info, warn};

use crate::{ControlCommand, FeedbackFrame, ActuatorType};

/// JSON log entry for actuator commands
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandLogEntry {
    pub timestamp_us: u64,
    pub actuator_id: u8,
    pub actuator_type: String,
    pub event_type: String, // "command"
    pub target_angle_rad: f32,
    pub target_velocity_rads: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque_nm: f32,
}

/// JSON log entry for actuator feedback
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FeedbackLogEntry {
    pub timestamp_us: u64,
    pub actuator_id: u8,
    pub actuator_type: String,
    pub event_type: String, // "feedback"
    pub actual_angle_rad: f32,
    pub actual_velocity_rads: f32,
    pub actual_torque_nm: f32,
    pub temperature_c: f32,
    pub fault_uncalibrated: bool,
    pub fault_hall_encoding: bool,
    pub fault_magnetic_encoding: bool,
    pub fault_over_temperature: bool,
    pub fault_overcurrent: bool,
    pub fault_undervoltage: bool,
}

/// Internal log message
#[derive(Debug, Clone)]
pub enum LogMessage {
    Command(CommandLogEntry),
    Feedback(FeedbackLogEntry),
    Flush,
    Shutdown,
}

/// High-performance JSON logger for actuator data
pub struct JsonLogger {
    tx: mpsc::Sender<LogMessage>,
    _handle: tokio::task::JoinHandle<()>,
}

impl JsonLogger {
    /// Create a new JSON logger with specified file path and buffer size
    pub async fn new<P: AsRef<Path>>(
        log_file_path: P,
        buffer_size: usize,
        flush_interval: Duration,
    ) -> eyre::Result<Self> {
        let (tx, mut rx) = mpsc::channel::<LogMessage>(buffer_size * 2);
        let log_file_path = log_file_path.as_ref().to_path_buf();
        
        info!("Creating JSON logger with file: {:?}", log_file_path);
        
        // Spawn background task to handle file writing
        let handle = tokio::spawn(async move {
            let mut file = match OpenOptions::new()
                .create(true)
                .append(true)
                .open(&log_file_path)
                .await
            {
                Ok(file) => file,
                Err(e) => {
                    error!("Failed to open log file {:?}: {}", log_file_path, e);
                    return;
                }
            };
            
            let mut buffer = Vec::with_capacity(buffer_size);
            let mut last_flush = Instant::now();
            
            info!("JSON logger started, writing to: {:?}", log_file_path);
            
            while let Some(msg) = rx.recv().await {
                match msg {
                    LogMessage::Command(entry) => {
                        if let Ok(json) = serde_json::to_string(&entry) {
                            buffer.push(json);
                        }
                    }
                    LogMessage::Feedback(entry) => {
                        if let Ok(json) = serde_json::to_string(&entry) {
                            buffer.push(json);
                        }
                    }
                    LogMessage::Flush => {
                        // Force flush
                        if !buffer.is_empty() {
                            if let Err(e) = flush_buffer(&mut file, &mut buffer).await {
                                error!("Failed to flush buffer: {}", e);
                            }
                            last_flush = Instant::now();
                        }
                    }
                    LogMessage::Shutdown => {
                        // Final flush and exit
                        if !buffer.is_empty() {
                            let _ = flush_buffer(&mut file, &mut buffer).await;
                        }
                        let _ = file.flush().await;
                        info!("JSON logger shutdown complete");
                        break;
                    }
                }
                
                // Auto-flush if buffer is full or time interval passed
                if buffer.len() >= buffer_size || last_flush.elapsed() >= flush_interval {
                    if !buffer.is_empty() {
                        if let Err(e) = flush_buffer(&mut file, &mut buffer).await {
                            error!("Failed to flush buffer: {}", e);
                        }
                        last_flush = Instant::now();
                    }
                }
            }
        });
        
        Ok(JsonLogger {
            tx,
            _handle: handle,
        })
    }
    
    /// Log a command sent to an actuator
    pub async fn log_command(
        &self,
        actuator_id: u8,
        actuator_type: ActuatorType,
        command: &ControlCommand,
    ) {
        let entry = CommandLogEntry {
            timestamp_us: get_timestamp_us(),
            actuator_id,
            actuator_type: format!("{:?}", actuator_type),
            event_type: "command".to_string(),
            target_angle_rad: command.target_angle,
            target_velocity_rads: command.target_velocity,
            kp: command.kp,
            kd: command.kd,
            torque_nm: command.torque,
        };
        
        if let Err(e) = self.tx.try_send(LogMessage::Command(entry)) {
            warn!("Failed to log command: {}", e);
        }
    }
    
    /// Log feedback received from an actuator
    pub async fn log_feedback(
        &self,
        actuator_id: u8,
        actuator_type: ActuatorType,
        feedback: &FeedbackFrame,
    ) {
        let entry = FeedbackLogEntry {
            timestamp_us: get_timestamp_us(),
            actuator_id,
            actuator_type: format!("{:?}", actuator_type),
            event_type: "feedback".to_string(),
            actual_angle_rad: feedback.angle,
            actual_velocity_rads: feedback.velocity,
            actual_torque_nm: feedback.torque,
            temperature_c: feedback.temperature,
            fault_uncalibrated: feedback.fault_uncalibrated,
            fault_hall_encoding: feedback.fault_hall_encoding,
            fault_magnetic_encoding: feedback.fault_magnetic_encoding,
            fault_over_temperature: feedback.fault_over_temperature,
            fault_overcurrent: feedback.fault_overcurrent,
            fault_undervoltage: feedback.fault_undervoltage,
        };
        
        if let Err(e) = self.tx.try_send(LogMessage::Feedback(entry)) {
            warn!("Failed to log feedback: {}", e);
        }
    }
    
    /// Force flush all buffered logs
    pub async fn flush(&self) {
        let _ = self.tx.try_send(LogMessage::Flush);
    }
    
    /// Shutdown the logger gracefully
    pub async fn shutdown(&self) {
        let _ = self.tx.try_send(LogMessage::Shutdown);
    }
}

/// Helper function to flush buffer to file
async fn flush_buffer(
    file: &mut tokio::fs::File,
    buffer: &mut Vec<String>,
) -> Result<(), std::io::Error> {
    for entry in buffer.iter() {
        file.write_all(entry.as_bytes()).await?;
        file.write_all(b"\n").await?;
    }
    file.flush().await?;
    buffer.clear();
    Ok(())
}

/// Get current timestamp in microseconds
fn get_timestamp_us() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}
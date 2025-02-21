use eyre::Result;
use robstride::robstride03::{RobStride03, RobStride03Command};
use robstride::SocketCanTransport;
use robstride::{Actuator, ControlCommand, Protocol, Transport, TxCommand, TypedCommandData};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tracing::{error, info, trace};
use tracing_subscriber::{fmt, EnvFilter};

#[tokio::main]
async fn main() -> Result<()> {
    // Set up logging
    let subscriber = fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .finish();
    tracing::subscriber::set_global_default(subscriber)?;

    // Create channel for sending commands
    let (tx, mut rx) = mpsc::channel(32);

    // Initialize CAN transport
    let transport = SocketCanTransport::new("can1".to_string()).await?;
    let mut transport_clone = transport.clone(); // Make this mutable

    // Spawn transport handling task
    let _transport_task = tokio::spawn(async move {
        info!("Starting transport handling task");
        loop {
            tokio::select! {
                // Handle outgoing messages
                Some(cmd) = rx.recv() => {
                    trace!("Processing outgoing command: {:?}", cmd);
                    match cmd {
                        TxCommand::Send { id, data } => {
                            if let Err(e) = transport_clone.send(id, &data).await {
                                error!("Transport sender error: {}", e);
                            }
                        }
                    }
                }
            }
        }
    });

    // Create motor instance (using ID 1 and host ID 0)
    let motor = RobStride03::new(33, 0, tx);

    // Enable the motor
    println!("Enabling motor...");
    motor.enable().await?;
    tokio::time::sleep(Duration::from_millis(500)).await;

    // Set some reasonable limits
    println!("Setting limits...");
    motor.set_max_torque(20.0).await?; // 20 Nm max torque
    motor.set_max_velocity(5.0).await?; // 5 rad/s max velocity

    // Create a position command (1 radian)
    let target_position = 0.0;
    println!("Moving to position: {} radians", target_position);

    let command = RobStride03Command {
        target_angle_rad: target_position,
        target_velocity_rads: 0.0,
        kp: 25.0, // Position gain
        kd: 5.0,  // Velocity gain
        torque_nm: 0.0,
    }
    .to_control_command();

    // Send the command
    motor.control(command).await?;

    // Wait for movement
    tokio::time::sleep(Duration::from_secs(2)).await;

    // Get feedback (position)
    motor.get_feedback().await?;

    // Disable the motor
    println!("Disabling motor...");
    motor.disable(false).await?;

    // Wait a moment before shutting down
    tokio::time::sleep(Duration::from_millis(100)).await;

    Ok(())
}

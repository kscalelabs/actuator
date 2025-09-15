use robstride::{
    robstride02::RobStride02, ActuatorConfiguration, ActuatorType, ControlConfig, StubTransport,
    Supervisor, TransportType,
};
use std::time::Duration;
use tracing::{error, info};
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() -> eyre::Result<()> {
    // Initialize tracing with specific filters
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    let mut supervisor = Supervisor::new(Duration::from_millis(1000))?;

    // Initialize SocketCAN transport
    // let socketcan = SocketCanTransport::new("can0".to_string()).await?;
    // supervisor
    //     .add_transport("socketcan".to_string(), TransportType::SocketCAN(socketcan))
    //     .await?;

    let stub = StubTransport::new("stub".to_string());
    supervisor
        .add_transport("stub".to_string(), TransportType::Stub(stub))
        .await?;

    // Spawn supervisor task
    let mut supervisor_runner = supervisor.clone_controller();
    let supervisor_handle = tokio::spawn(async move {
        info!("Starting supervisor task");
        if let Err(e) = supervisor_runner.run(Duration::from_millis(500)).await {
            error!("Supervisor task failed: {}", e);
        }
    });

    // // Scan for actuators on the bus
    // let desired_actuator_types: &[(u8, ActuatorConfiguration)] = &[
    //     (1, ActuatorConfiguration {
    //         actuator_type: ActuatorType::RobStride04,
    //         ..Default::default()
    //     }),
    // ];

    // let discovered_ids = supervisor
    //     .scan_bus(0xFD, "socketcan", &desired_actuator_types)
    //     .await?;

    // debug!("Discovered IDs: {:?}", discovered_ids);

    supervisor
        .add_actuator(
            Box::new(RobStride02::new(
                1,
                0xFE,
                supervisor.get_transport_tx("stub").await?,
            )),
            ActuatorConfiguration {
                actuator_type: ActuatorType::RobStride02,
                ..Default::default()
            },
        )
        .await;

    // Configure actuator 1
    let config = ControlConfig {
        kp: 5.0,
        kd: 0.1,
        max_torque: Some(2.0),
        max_velocity: Some(1.0),
        max_current: Some(10.0),
    };

    // for id in &discovered_ids {
    //     supervisor.configure(*id, config.clone()).await?;
    //     // supervisor.zero(*id).await?;
    // }

    // Enable actuator 1 AFTER spawning the supervisor

    // debug!("Enabling actuator 1");
    // supervisor.enable(1).await?;

    // Start control loop with original supervisor instance
    // let mut target_angle = std::f32::consts::FRAC_PI_2;

    // Create a ctrl-c handler
    let (tx, mut rx) = tokio::sync::mpsc::channel(1);
    let tx_clone = tx.clone();

    tokio::spawn(async move {
        if let Err(e) = tokio::signal::ctrl_c().await {
            error!("Failed to listen for ctrl-c: {}", e);
        }
        let _ = tx_clone.send(()).await;
    });

    info!("Configuring actuator 1");
    supervisor.configure(1, config.clone()).await?;
    supervisor.enable(1).await?;

    info!("Starting control loop, press Ctrl+C to exit");
    loop {
        tokio::select! {
            _ = rx.recv() => {
                info!("Received shutdown signal");
                break;
            }
            _ = tokio::time::sleep(Duration::from_millis(500)) => {
                supervisor.command(1, 1.0, 0.0, 0.0).await?;

                // for id in &discovered_ids {
                //     let feedback = supervisor.get_feedback(*id).await?;
                //     if let Some((feedback, _ts)) = feedback {
                //         debug!("Motor {}: angle = {:.3} rad ({:.1}°)",
                //             id,
                //             feedback.angle,
                //             feedback.angle.to_degrees()
                //         );
                //     }
                // }
            }
        }
    }

    // Clean shutdown
    info!("Shutting down supervisor...");
    supervisor_handle.abort();
    // for id in &discovered_ids {
    //     if let Err(e) = supervisor.disable(*id, false).await {
    //         error!("Failed to disable actuator {}: {}", id, e);
    //     }
    //     debug!("Disabled actuator {}", id);
    // }

    Ok(())
}

use robstride::{ActuatorConfiguration, ActuatorType, StubTransport, Supervisor, TransportType};
use std::time::Duration;
use tracing::info;
use tracing_subscriber::EnvFilter;

fn main() -> eyre::Result<()> {
    // Initialize tracing with specific filters
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::from_default_env()
                .add_directive("polling=off".parse().unwrap())
                .add_directive("async_io=off".parse().unwrap()),
        )
        .init();

    let supervisor = Supervisor::new()?;

    // Initialize SocketCAN transport
    // let socketcan = SocketCanTransport::new("can0".to_string())?;
    // supervisor
    //     .add_transport("socketcan".to_string(), TransportType::SocketCAN(socketcan))?;

    let stub = StubTransport::new();
    supervisor.add_transport("stub".to_string(), TransportType::Stub(stub))?;

    // Add a simple actuator for testing
    supervisor.add_actuator(
        1,
        ActuatorType::RobStride02,
        ActuatorConfiguration {
            actuator_type: ActuatorType::RobStride02,
            ..Default::default()
        },
    )?;

    info!("Supervisor running with stub transport");
    info!("Press Ctrl+C to exit");

    // Simple synchronous loop
    loop {
        std::thread::sleep(Duration::from_millis(100));

        // Check actuator status
        let states = supervisor.get_actuators_state(vec![1]);
        if let Some(state) = states?.first() {
            if state.ready {
                info!(
                    "Actuator 1: online={}, position={:.2}°, velocity={:.2}°/s, torque={:.2}Nm",
                    state.ready,
                    state
                        .feedback
                        .as_ref()
                        .map(|f| f.angle.to_degrees())
                        .unwrap_or(0.0),
                    state
                        .feedback
                        .as_ref()
                        .map(|f| f.velocity.to_degrees())
                        .unwrap_or(0.0),
                    state.feedback.as_ref().map(|f| f.torque).unwrap_or(0.0)
                );
            } else {
                info!("Actuator 1: offline");
            }
        }
    }
}

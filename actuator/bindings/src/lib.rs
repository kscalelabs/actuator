use pyo3::prelude::PyErr;
use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};
#[cfg(target_os = "linux")]
use robstride::SocketCanTransport;
use robstride::{
    ActuatorConfiguration, ActuatorType, CH341Transport, ControlCommand, StubTransport, Supervisor,
    Transport, TransportType,
};
use std::sync::{Arc, Mutex};
use std::time::Duration;

struct ErrReportWrapper(eyre::Report);

impl From<eyre::Report> for ErrReportWrapper {
    fn from(err: eyre::Report) -> Self {
        ErrReportWrapper(err)
    }
}

impl From<ErrReportWrapper> for PyErr {
    fn from(err: ErrReportWrapper) -> PyErr {
        PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(err.0.to_string())
    }
}

#[pyfunction]
#[gen_stub_pyfunction]
fn get_version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct RobstrideActuatorCommand {
    #[pyo3(get, set)]
    actuator_id: u32,
    #[pyo3(get, set)]
    position: Option<f64>,
    #[pyo3(get, set)]
    velocity: Option<f64>,
    #[pyo3(get, set)]
    torque: Option<f64>,
}

#[pymethods]
impl RobstrideActuatorCommand {
    #[new]
    fn new(actuator_id: u32) -> Self {
        Self {
            actuator_id,
            position: None,
            velocity: None,
            torque: None,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct RobstrideConfigureRequest {
    #[pyo3(get, set)]
    actuator_id: u32,
    #[pyo3(get, set)]
    kp: Option<f64>,
    #[pyo3(get, set)]
    kd: Option<f64>,
    #[pyo3(get, set)]
    max_torque: Option<f64>,
    #[pyo3(get, set)]
    torque_enabled: Option<bool>,
    #[pyo3(get, set)]
    zero_position: Option<bool>,
    #[pyo3(get, set)]
    new_actuator_id: Option<u32>,
}

#[pymethods]
impl RobstrideConfigureRequest {
    #[new]
    fn new(actuator_id: u32) -> Self {
        Self {
            actuator_id,
            kp: None,
            kd: None,
            max_torque: None,
            torque_enabled: None,
            zero_position: None,
            new_actuator_id: None,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct RobstrideActuatorState {
    #[pyo3(get)]
    actuator_id: u32,
    #[pyo3(get)]
    online: bool,
    #[pyo3(get)]
    position: Option<f64>,
    #[pyo3(get)]
    velocity: Option<f64>,
    #[pyo3(get)]
    torque: Option<f64>,
    #[pyo3(get)]
    temperature: Option<f64>,
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct RobstrideActuatorConfig {
    #[pyo3(get, set)]
    actuator_type: u8,
    #[pyo3(get, set)]
    max_angle_change: Option<f64>,
    #[pyo3(get, set)]
    max_velocity: Option<f64>,
}

#[pymethods]
impl RobstrideActuatorConfig {
    #[new]
    fn new(actuator_type: u8) -> Self {
        Self {
            actuator_type,
            max_angle_change: None,
            max_velocity: None,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
pub struct CH341TransportWrapper {
    transport: std::sync::Mutex<CH341Transport>,
}

impl CH341TransportWrapper {
    fn get_transport(&self) -> Result<CH341Transport, ErrReportWrapper> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.clone())
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl CH341TransportWrapper {
    #[new]
    fn new(port_name: String) -> PyResult<Self> {
        let transport = CH341Transport::new(port_name).map_err(ErrReportWrapper)?;
        Ok(Self {
            transport: std::sync::Mutex::new(transport),
        })
    }

    /// Send data over the transport
    fn send(&self, id: u32, data: Vec<u8>) -> PyResult<bool> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.send(id, &data) {
            Ok(()) => Ok(true),
            Err(e) => Err(ErrReportWrapper(e.into()).into()),
        }
    }

    /// Receive data from the transport
    fn recv(&self) -> PyResult<Option<(u32, Vec<u8>)>> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.recv() {
            Ok((id, data)) => Ok(Some((id, data))),
            Err(_) => Ok(None), // No data available or error
        }
    }

    /// Clear the receive buffer
    fn clear_buffer(&self) -> PyResult<()> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        transport
            .clear_buffer()
            .map_err(|e| ErrReportWrapper(e.into()))?;
        Ok(())
    }

    /// Get the port name
    fn port(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.port())
    }

    /// Get the transport kind
    fn kind(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.kind().to_string())
    }
}

#[cfg(target_os = "linux")]
#[gen_stub_pyclass]
#[pyclass]
pub struct SocketCanTransportWrapper {
    transport: std::sync::Mutex<SocketCanTransport>,
}

#[cfg(target_os = "linux")]
impl SocketCanTransportWrapper {
    fn get_transport(&self) -> Result<SocketCanTransport, ErrReportWrapper> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.clone())
    }
}

#[cfg(target_os = "linux")]
#[gen_stub_pymethods]
#[pymethods]
impl SocketCanTransportWrapper {
    #[new]
    fn new(interface_name: String) -> PyResult<Self> {
        let transport = SocketCanTransport::new(interface_name).map_err(ErrReportWrapper)?;
        Ok(Self {
            transport: std::sync::Mutex::new(transport),
        })
    }

    /// Send data over the transport
    fn send(&self, id: u32, data: Vec<u8>) -> PyResult<bool> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.send(id, &data) {
            Ok(()) => Ok(true),
            Err(e) => Err(ErrReportWrapper(e.into()).into()),
        }
    }

    /// Receive data from the transport
    fn recv(&self) -> PyResult<Option<(u32, Vec<u8>)>> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.recv() {
            Ok((id, data)) => Ok(Some((id, data))),
            Err(_) => Ok(None), // No data available or error
        }
    }

    /// Clear the receive buffer
    fn clear_buffer(&self) -> PyResult<()> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        transport
            .clear_buffer()
            .map_err(|e| ErrReportWrapper(e.into()))?;
        Ok(())
    }

    /// Get the port name
    fn port(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.port())
    }

    /// Get the transport kind
    fn kind(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.kind().to_string())
    }
}

#[gen_stub_pyclass]
#[pyclass]
pub struct StubTransportWrapper {
    transport: std::sync::Mutex<StubTransport>,
}

impl StubTransportWrapper {
    fn get_transport(&self) -> Result<StubTransport, ErrReportWrapper> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.clone())
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl StubTransportWrapper {
    #[new]
    fn new(port_name: String) -> Self {
        Self {
            transport: std::sync::Mutex::new(StubTransport::new(port_name)),
        }
    }

    /// Send data over the transport
    fn send(&self, id: u32, data: Vec<u8>) -> PyResult<bool> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.send(id, &data) {
            Ok(()) => Ok(true),
            Err(e) => Err(ErrReportWrapper(e.into()).into()),
        }
    }

    /// Receive data from the transport
    fn recv(&self) -> PyResult<Option<(u32, Vec<u8>)>> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        match transport.recv() {
            Ok((id, data)) => Ok(Some((id, data))),
            Err(_) => Ok(None), // No data available or error
        }
    }

    /// Clear the receive buffer
    fn clear_buffer(&self) -> PyResult<()> {
        let mut transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        transport
            .clear_buffer()
            .map_err(|e| ErrReportWrapper(e.into()))?;
        Ok(())
    }

    /// Get the port name
    fn port(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.port())
    }

    /// Get the transport kind
    fn kind(&self) -> PyResult<String> {
        let transport = self.transport.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire transport lock: {}", e))
        })?;
        Ok(transport.kind().to_string())
    }
}

#[gen_stub_pyclass]
#[pyclass]
struct RobstrideActuator {
    supervisor: Arc<Mutex<Supervisor>>,
}

#[gen_stub_pymethods]
#[pymethods]
impl RobstrideActuator {
    #[new]
    fn new(
        transports: Vec<Py<PyAny>>,
        py_actuators_config: Vec<(u8, RobstrideActuatorConfig)>,
        py: Python,
    ) -> PyResult<Self> {
        let actuators_config: Vec<(u8, ActuatorConfiguration)> = py_actuators_config
            .into_iter()
            .map(|(id, config)| (id, config.into()))
            .collect();

        let mut supervisor = Supervisor::new(Duration::from_secs(1)).map_err(ErrReportWrapper)?;

        for transport_obj in &transports {
            let transport_type = Self::extract_transport_type(transport_obj, py)
                .map_err(|e| ErrReportWrapper(eyre::eyre!("Transport extraction failed: {}", e)))?;
            let port_name = transport_type.port();
            supervisor
                .add_transport(port_name, transport_type)
                .map_err(ErrReportWrapper)?;
        }

        // Scan for motors
        for transport_obj in &transports {
            let transport_type = Self::extract_transport_type(transport_obj, py)
                .map_err(|e| ErrReportWrapper(eyre::eyre!("Transport extraction failed: {}", e)))?;
            let port_name = transport_type.port();
            let discovered_ids = supervisor
                .scan_bus(0xFD, &port_name, &actuators_config)
                .map_err(ErrReportWrapper)?;
            for (motor_id, _) in &actuators_config {
                if !discovered_ids.contains(motor_id) {
                    tracing::warn!("Configured motor not found - ID: {}", motor_id);
                }
            }
        }

        Ok(RobstrideActuator {
            supervisor: Arc::new(Mutex::new(supervisor)),
        })
    }

    /// Update the setpoints for the actuators
    fn command_actuators(&self, commands: Vec<RobstrideActuatorCommand>) -> PyResult<Vec<bool>> {
        let supervisor = self.supervisor.lock().map_err(|e| {
            ErrReportWrapper(eyre::eyre!("Failed to acquire supervisor lock: {}", e))
        })?;

        let control_commands: Vec<ControlCommand> = commands
            .into_iter()
            .map(|cmd| ControlCommand {
                target_angle: cmd.position.map(|p| p.to_radians() as f32).unwrap_or(0.0),
                target_velocity: cmd.velocity.map(|v| v.to_radians() as f32).unwrap_or(0.0),
                kp: 0.0,
                kd: 0.0,
                torque: cmd.torque.map(|t| t as f32).unwrap_or(0.0),
            })
            .collect();

        let results = supervisor.command_actuators(control_commands);
        Ok(results)
    }

    /// Configure the actuator
    fn configure_actuator(&self, config: RobstrideConfigureRequest) -> PyResult<bool> {
        // For now, just return success
        // In a real implementation, you would configure the actuator here
        Ok(true)
    }

    /// Get the state of the actuators
    fn get_actuators_state(&self, actuator_ids: Vec<u32>) -> PyResult<Vec<RobstrideActuatorState>> {
        let supervisor = self.supervisor.lock().unwrap();
        let states =
            supervisor.get_actuators_state(actuator_ids.iter().map(|&id| id as u8).collect());

        let mut responses = vec![];
        for (i, state) in states.iter().enumerate() {
            responses.push(RobstrideActuatorState {
                actuator_id: actuator_ids[i],
                online: state.ready,
                position: state.feedback.as_ref().map(|f| f.angle.to_degrees() as f64),
                velocity: state
                    .feedback
                    .as_ref()
                    .map(|f| f.velocity.to_degrees() as f64),
                torque: state.feedback.as_ref().map(|f| f.torque as f64),
                temperature: Some(25.0), // Default temperature
            });
        }
        Ok(responses)
    }
}

impl RobstrideActuator {
    fn extract_transport_type(
        transport_obj: &Py<PyAny>,
        py: Python,
    ) -> Result<TransportType, PyErr> {
        // Try to extract CH341Transport
        if let Ok(ch341_wrapper) = transport_obj.extract::<PyRef<CH341TransportWrapper>>(py) {
            return Ok(TransportType::CH341(ch341_wrapper.get_transport()?));
        }

        // Try to extract SocketCanTransport (Linux only)
        #[cfg(target_os = "linux")]
        if let Ok(socketcan_wrapper) = transport_obj.extract::<PyRef<SocketCanTransportWrapper>>(py)
        {
            return Ok(TransportType::SocketCAN(socketcan_wrapper.get_transport()?));
        }

        // Try to extract StubTransport
        if let Ok(stub_wrapper) = transport_obj.extract::<PyRef<StubTransportWrapper>>(py) {
            return Ok(TransportType::Stub(stub_wrapper.get_transport()?));
        }

        Err(PyErr::new::<pyo3::exceptions::PyTypeError, _>(
            "Invalid transport object. Must be one of: CH341TransportWrapper, SocketCanTransportWrapper, or StubTransportWrapper"
        ))
    }
}

impl From<RobstrideActuatorConfig> for robstride::ActuatorConfiguration {
    fn from(config: RobstrideActuatorConfig) -> Self {
        Self {
            actuator_type: match config.actuator_type {
                0 => ActuatorType::RobStride00,
                1 => ActuatorType::RobStride01,
                2 => ActuatorType::RobStride02,
                3 => ActuatorType::RobStride03,
                4 => ActuatorType::RobStride04,
                _ => ActuatorType::RobStride00,
            },
            max_angle_change: config.max_angle_change.map(|v| v as f32),
            max_velocity: config.max_velocity.map(|v| v as f32),
            command_rate_hz: Some(100.0f32),
        }
    }
}

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(get_version, m)?)?;
    m.add_class::<RobstrideActuator>()?;
    m.add_class::<RobstrideActuatorCommand>()?;
    m.add_class::<RobstrideConfigureRequest>()?;
    m.add_class::<RobstrideActuatorState>()?;
    m.add_class::<RobstrideActuatorConfig>()?;
    m.add_class::<CH341TransportWrapper>()?;
    #[cfg(target_os = "linux")]
    m.add_class::<SocketCanTransportWrapper>()?;
    m.add_class::<StubTransportWrapper>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

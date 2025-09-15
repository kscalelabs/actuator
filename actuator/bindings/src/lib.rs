use pyo3::prelude::PyErr;
use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};
#[cfg(target_os = "linux")]
use robstride::SocketCanTransport;
use robstride::{
    ActuatorConfiguration, ActuatorType, CH341Transport, ControlConfig, StubTransport, Supervisor,
    Transport, TransportType,
};
use std::sync::Arc;
use std::time::Duration;
use tokio::runtime::Runtime;
use tokio::sync::Mutex;

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

#[gen_stub_pymethods]
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

#[gen_stub_pymethods]
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

#[gen_stub_pymethods]
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
    transport: CH341Transport,
}

impl CH341TransportWrapper {
    fn get_transport(&self) -> CH341Transport {
        self.transport.clone()
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl CH341TransportWrapper {
    #[new]
    fn new(port_name: String) -> PyResult<Self> {
        let rt = Runtime::new().map_err(|e| ErrReportWrapper(e.into()))?;
        let transport = rt.block_on(async {
            CH341Transport::new(port_name)
                .await
                .map_err(ErrReportWrapper)
        })?;
        Ok(Self { transport })
    }
}

#[cfg(target_os = "linux")]
#[gen_stub_pyclass]
#[pyclass]
pub struct SocketCanTransportWrapper {
    transport: SocketCanTransport,
}

#[cfg(target_os = "linux")]
impl SocketCanTransportWrapper {
    fn get_transport(&self) -> SocketCanTransport {
        self.transport.clone()
    }
}

#[cfg(target_os = "linux")]
#[gen_stub_pymethods]
#[pymethods]
impl SocketCanTransportWrapper {
    #[new]
    fn new(interface_name: String) -> PyResult<Self> {
        let rt = Runtime::new().map_err(|e| ErrReportWrapper(e.into()))?;
        let transport = rt.block_on(async {
            SocketCanTransport::new(interface_name)
                .await
                .map_err(ErrReportWrapper)
        })?;
        Ok(Self { transport })
    }
}

#[gen_stub_pyclass]
#[pyclass]
pub struct StubTransportWrapper {
    transport: StubTransport,
}

impl StubTransportWrapper {
    fn get_transport(&self) -> StubTransport {
        self.transport.clone()
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl StubTransportWrapper {
    #[new]
    fn new(port_name: String) -> Self {
        Self {
            transport: StubTransport::new(port_name),
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
struct RobstrideActuator {
    supervisor: Arc<Mutex<Supervisor>>,
    rt: Runtime,
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

        let rt = Runtime::new().map_err(|e| ErrReportWrapper(e.into()))?;

        let supervisor = rt.block_on(async {
            let mut supervisor =
                Supervisor::new(Duration::from_secs(1)).map_err(ErrReportWrapper)?;

            for transport_obj in &transports {
                let transport_type =
                    Self::extract_transport_type(transport_obj, py).map_err(|e| {
                        ErrReportWrapper(eyre::eyre!("Transport extraction failed: {}", e))
                    })?;
                let port_name = transport_type.port();
                supervisor
                    .add_transport(port_name, transport_type)
                    .await
                    .map_err(ErrReportWrapper)?;
            }

            // Scan for motors
            for transport_obj in &transports {
                let transport_type =
                    Self::extract_transport_type(transport_obj, py).map_err(|e| {
                        ErrReportWrapper(eyre::eyre!("Transport extraction failed: {}", e))
                    })?;
                let port_name = transport_type.port();
                let discovered_ids = supervisor
                    .scan_bus(0xFD, &port_name, &actuators_config)
                    .await
                    .map_err(ErrReportWrapper)?;
                for (motor_id, _) in &actuators_config {
                    if !discovered_ids.contains(motor_id) {
                        tracing::warn!("Configured motor not found - ID: {}", motor_id);
                    }
                }
            }

            Ok::<Supervisor, ErrReportWrapper>(supervisor)
        })?;

        Ok(RobstrideActuator {
            supervisor: Arc::new(Mutex::new(supervisor)),
            rt,
        })
    }

    fn command_actuators(&self, commands: Vec<RobstrideActuatorCommand>) -> PyResult<Vec<bool>> {
        self.rt.block_on(async {
            let mut results = vec![];
            let mut supervisor = self.supervisor.lock().await;

            for cmd in commands {
                match supervisor
                    .command(
                        cmd.actuator_id as u8,
                        cmd.position.map(|p| p.to_radians() as f32).unwrap_or(0.0),
                        cmd.velocity.map(|v| v.to_radians() as f32).unwrap_or(0.0),
                        cmd.torque.map(|t| t as f32).unwrap_or(0.0),
                    )
                    .await
                {
                    Ok(_) => results.push(true),
                    Err(_) => results.push(false),
                }
            }
            Ok(results)
        })
    }

    fn configure_actuator(&self, config: RobstrideConfigureRequest) -> PyResult<bool> {
        self.rt.block_on(async {
            let mut supervisor = self.supervisor.lock().await;

            let control_config = ControlConfig {
                kp: config.kp.unwrap_or(0.0) as f32,
                kd: config.kd.unwrap_or(0.0) as f32,
                max_torque: Some(config.max_torque.unwrap_or(2.0) as f32),
                max_velocity: Some(5.0),
                max_current: Some(10.0),
            };

            supervisor
                .configure(config.actuator_id as u8, control_config)
                .await
                .map_err(ErrReportWrapper)?;

            if let Some(torque_enabled) = config.torque_enabled {
                if torque_enabled {
                    supervisor
                        .enable(config.actuator_id as u8)
                        .await
                        .map_err(ErrReportWrapper)?;
                } else {
                    supervisor
                        .disable(config.actuator_id as u8, true)
                        .await
                        .map_err(ErrReportWrapper)?;
                }
            }

            if let Some(true) = config.zero_position {
                supervisor
                    .zero(config.actuator_id as u8)
                    .await
                    .map_err(ErrReportWrapper)?;
            }

            if let Some(new_id) = config.new_actuator_id {
                supervisor
                    .change_id(config.actuator_id as u8, new_id as u8)
                    .await
                    .map_err(ErrReportWrapper)?;
            }

            Ok(true)
        })
    }

    fn get_actuators_state(&self, actuator_ids: Vec<u32>) -> PyResult<Vec<RobstrideActuatorState>> {
        self.rt.block_on(async {
            let mut responses = vec![];
            let supervisor = self.supervisor.lock().await;

            for id in actuator_ids {
                if let Ok(Some((feedback, ts))) = supervisor.get_feedback(id as u8).await {
                    responses.push(RobstrideActuatorState {
                        actuator_id: id,
                        online: ts.elapsed().unwrap_or(Duration::from_secs(1))
                            < Duration::from_secs(1),
                        position: Some(feedback.angle.to_degrees() as f64),
                        velocity: Some(feedback.velocity.to_degrees() as f64),
                        torque: Some(feedback.torque as f64),
                        temperature: Some(feedback.temperature as f64),
                    });
                }
            }
            Ok(responses)
        })
    }
}

impl RobstrideActuator {
    fn extract_transport_type(
        transport_obj: &Py<PyAny>,
        py: Python,
    ) -> Result<TransportType, PyErr> {
        // Try to extract CH341Transport
        if let Ok(ch341_wrapper) = transport_obj.extract::<PyRef<CH341TransportWrapper>>(py) {
            return Ok(TransportType::CH341(ch341_wrapper.get_transport()));
        }

        // Try to extract SocketCanTransport (Linux only)
        #[cfg(target_os = "linux")]
        if let Ok(socketcan_wrapper) = transport_obj.extract::<PyRef<SocketCanTransportWrapper>>(py)
        {
            return Ok(TransportType::SocketCAN(socketcan_wrapper.get_transport()));
        }

        // Try to extract StubTransport
        if let Ok(stub_wrapper) = transport_obj.extract::<PyRef<StubTransportWrapper>>(py) {
            return Ok(TransportType::Stub(stub_wrapper.get_transport()));
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

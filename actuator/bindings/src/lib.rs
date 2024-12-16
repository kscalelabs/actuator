use pyo3::prelude::*;
use pyo3::prelude::PyErr;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};
use robstride::{
    ActuatorType, ActuatorConfiguration, Supervisor, TransportType,
    CH341Transport, ControlConfig, SocketCanTransport,
};
use std::sync::Arc;
use tokio::sync::Mutex;
use std::time::Duration;
use tokio::runtime::Runtime;

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
struct PyRobstrideActuatorCommand {
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
impl PyRobstrideActuatorCommand {
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
struct PyRobstrideConfigureRequest {
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
impl PyRobstrideConfigureRequest {
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
struct PyRobstrideActuatorState {
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
struct PyRobstrideActuatorConfig {
    #[pyo3(get, set)]
    actuator_type: u8,
    #[pyo3(get, set)]
    max_angle_change: Option<f64>,
    #[pyo3(get, set)]
    max_velocity: Option<f64>,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideActuatorConfig {
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
struct PyRobstrideActuator {
    supervisor: Arc<Mutex<Supervisor>>,
    rt: Runtime,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideActuator {
    #[new]
    fn new(
        ports: Vec<String>,
        py_actuators_config: Vec<(u8, PyRobstrideActuatorConfig)>,
        polling_interval: f64,
    ) -> PyResult<Self> {
        let actuators_config: Vec<(u8, ActuatorConfiguration)> = py_actuators_config
            .into_iter()
            .map(|(id, config)| (id, config.into()))
            .collect();

        let rt = Runtime::new().map_err(|e| ErrReportWrapper(e.into()))?;
        
        let supervisor = rt.block_on(async {
            let mut supervisor = Supervisor::new(Duration::from_secs(1))
                .map_err(|e| ErrReportWrapper(e))?;

            for port in &ports {
                if port.starts_with("/dev/tty") {
                    let serial = CH341Transport::new(port.clone()).await
                        .map_err(|e| ErrReportWrapper(e))?;
                    supervisor
                        .add_transport(port.clone(), TransportType::CH341(serial))
                        .await
                        .map_err(|e| ErrReportWrapper(e))?;
                } else if port.starts_with("can") {
                    let can = SocketCanTransport::new(port.clone()).await
                        .map_err(|e| ErrReportWrapper(e))?;
                    supervisor
                        .add_transport(port.clone(), TransportType::SocketCAN(can))
                        .await
                        .map_err(|e| ErrReportWrapper(e))?;
                } else {
                    return Err(ErrReportWrapper(eyre::eyre!("Invalid port: {}", port)));
                }
            }

            // Scan for motors
            for port in &ports {
                let discovered_ids = supervisor.scan_bus(0xFD, port, &actuators_config).await
                    .map_err(|e| ErrReportWrapper(e))?;
                for (motor_id, _) in &actuators_config {
                    if !discovered_ids.contains(motor_id) {
                        tracing::warn!("Configured motor not found - ID: {}", motor_id);
                    }
                }
            }

            Ok(supervisor)
        })?;

        Ok(PyRobstrideActuator {
            supervisor: Arc::new(Mutex::new(supervisor)),
            rt,
        })
    }

    fn command_actuators(&self, commands: Vec<PyRobstrideActuatorCommand>) -> PyResult<Vec<bool>> {
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

    fn configure_actuator(&self, config: PyRobstrideConfigureRequest) -> PyResult<bool> {
        self.rt.block_on(async {
            let mut supervisor = self.supervisor.lock().await;
            
            let control_config = ControlConfig {
                kp: config.kp.unwrap_or(0.0) as f32,
                kd: config.kd.unwrap_or(0.0) as f32,
                max_torque: Some(config.max_torque.unwrap_or(2.0) as f32),
                max_velocity: Some(5.0),
                max_current: Some(10.0),
            };

            let _result = supervisor.configure(config.actuator_id as u8, control_config).await
                .map_err(|e| ErrReportWrapper(e))?;

            if let Some(torque_enabled) = config.torque_enabled {
                if torque_enabled {
                    supervisor.enable(config.actuator_id as u8).await
                        .map_err(|e| ErrReportWrapper(e))?;
                } else {
                    supervisor.disable(config.actuator_id as u8, true).await
                        .map_err(|e| ErrReportWrapper(e))?;
                }
            }

            if let Some(true) = config.zero_position {
                supervisor.zero(config.actuator_id as u8).await
                    .map_err(|e| ErrReportWrapper(e))?;
            }

            if let Some(new_id) = config.new_actuator_id {
                supervisor
                    .change_id(config.actuator_id as u8, new_id as u8)
                    .await
                    .map_err(|e| ErrReportWrapper(e))?;
            }

            Ok(true)
        })
    }

    fn get_actuators_state(&self, actuator_ids: Vec<u32>) -> PyResult<Vec<PyRobstrideActuatorState>> {
        self.rt.block_on(async {
            let mut responses = vec![];
            let supervisor = self.supervisor.lock().await;
            
            for id in actuator_ids {
                if let Ok(Some((feedback, ts))) = supervisor.get_feedback(id as u8).await {
                    responses.push(PyRobstrideActuatorState {
                        actuator_id: id,
                        online: ts.elapsed().unwrap_or(Duration::from_secs(1)) < Duration::from_secs(1),
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

impl From<PyRobstrideActuatorConfig> for robstride::ActuatorConfiguration {
    fn from(config: PyRobstrideActuatorConfig) -> Self {
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
        }
    }
}

#[pymodule]
fn robstride_bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(get_version, m)?)?;
    m.add_class::<PyRobstrideActuator>()?;
    m.add_class::<PyRobstrideActuatorCommand>()?;
    m.add_class::<PyRobstrideConfigureRequest>()?;
    m.add_class::<PyRobstrideActuatorState>()?;
    m.add_class::<PyRobstrideActuatorConfig>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

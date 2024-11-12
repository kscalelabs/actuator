use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pyfunction, gen_stub_pymethods};
use robstride::{
    motor_mode_from_str as robstride_motor_mode_from_str,
    motor_type_from_str as robstride_motor_type_from_str, MotorConfig as RobstrideMotorConfig,
    MotorControlParams as RobstrideMotorControlParams, MotorFeedback as RobstrideMotorFeedback,
    MotorType as RobstrideMotorType, Motors as RobstrideMotors,
    MotorsSupervisor as RobstrideMotorsSupervisor, ROBSTRIDE_CONFIGS as RobstrideDefaultConfigs,
};
use std::collections::HashMap;
use std::hash::{Hash, Hasher};

#[pyfunction]
#[gen_stub_pyfunction]
fn get_version() -> String {
    env!("CARGO_PKG_VERSION").to_string()
}

#[gen_stub_pyclass]
#[pyclass]
struct PyRobstrideMotors {
    inner: RobstrideMotors,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideMotors {
    #[new]
    #[pyo3(signature = (port_name, motor_infos, verbose = false))]
    fn new(port_name: String, motor_infos: HashMap<u8, String>, verbose: bool) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = robstride_motor_type_from_str(type_str.as_str())?;
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let motors = RobstrideMotors::new(&port_name, &motor_infos, verbose)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(PyRobstrideMotors { inner: motors })
    }

    fn send_get_mode(&mut self) -> PyResult<HashMap<u8, String>> {
        self.inner
            .send_get_mode()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, format!("{:?}", v))))
            .collect()
    }

    fn set_torque_limit(&mut self, motor_id: u8, torque_limit: f32) -> PyResult<f32> {
        self.inner.set_torque_limit(motor_id, torque_limit).map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(torque_limit)
    }

    fn set_speed_limit(&mut self, motor_id: u8, speed_limit: f32) -> PyResult<f32> {
        self.inner.set_speed_limit(motor_id, speed_limit).map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(speed_limit)
    }

    fn set_current_limit(&mut self, motor_id: u8, current_limit: f32) -> PyResult<f32> {
        self.inner.set_current_limit(motor_id, current_limit).map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        Ok(current_limit)
    }

    fn send_resets(&mut self) -> PyResult<()> {
        self.inner
            .send_resets()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn send_starts(&mut self) -> PyResult<()> {
        self.inner
            .send_starts()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn send_motor_controls(
        &mut self,
        motor_controls: HashMap<u8, PyRobstrideMotorControlParams>,
        serial: bool,
    ) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        let motor_controls: HashMap<u8, RobstrideMotorControlParams> = motor_controls
            .into_iter()
            .map(|(k, v)| (k, v.into()))
            .collect();

        self.inner
            .send_motor_controls(&motor_controls, serial)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!("PyRobstrideMotors"))
    }

    #[staticmethod]
    fn get_default_configs() -> PyResult<HashMap<PyRobstrideMotorType, PyRobstrideMotorConfig>> {
        Ok(RobstrideDefaultConfigs
            .iter()
            .map(|(motor_type, config)| ((*motor_type).into(), (*config).into()))
            .collect())
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct PyRobstrideMotorFeedback {
    #[pyo3(get)]
    can_id: u8,
    #[pyo3(get)]
    position: f32,
    #[pyo3(get)]
    velocity: f32,
    #[pyo3(get)]
    torque: f32,
    #[pyo3(get)]
    mode: String,
    #[pyo3(get)]
    faults: u16,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideMotorFeedback {
    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "PyRobstrideMotorFeedback(can_id={}, position={:.2}, velocity={:.2}, torque={:.2}, mode='{}', faults={})",
            self.can_id, self.position, self.velocity, self.torque, self.mode, self.faults
        ))
    }

    #[staticmethod]
    fn create_feedback(
        can_id: u8,
        position: f32,
        velocity: f32,
        torque: f32,
        mode: String,
        faults: u16,
    ) -> PyResult<Self> {
        let feedback = RobstrideMotorFeedback {
            can_id,
            position,
            velocity,
            torque,
            mode: robstride_motor_mode_from_str(mode.as_str())?,
            faults,
        };

        Ok(feedback.into())
    }
}

impl From<RobstrideMotorFeedback> for PyRobstrideMotorFeedback {
    fn from(feedback: RobstrideMotorFeedback) -> Self {
        PyRobstrideMotorFeedback {
            can_id: feedback.can_id,
            position: feedback.position,
            velocity: feedback.velocity,
            torque: feedback.torque,
            mode: format!("{:?}", feedback.mode),
            faults: feedback.faults,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(FromPyObject)]
struct PyRobstrideMotorControlParams {
    #[pyo3(get, set)]
    position: f32,
    #[pyo3(get, set)]
    velocity: f32,
    #[pyo3(get, set)]
    kp: f32,
    #[pyo3(get, set)]
    kd: f32,
    #[pyo3(get, set)]
    torque: f32,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideMotorControlParams {
    #[new]
    fn new(position: f32, velocity: f32, kp: f32, kd: f32, torque: f32) -> Self {
        PyRobstrideMotorControlParams {
            position,
            velocity,
            kp,
            kd,
            torque,
        }
    }

    fn __repr__(&self) -> PyResult<String> {
        Ok(format!(
            "PyRobstrideMotorControlParams(position={:.2}, velocity={:.2}, kp={:.2}, kd={:.2}, torque={:.2})",
            self.position, self.velocity, self.kp, self.kd, self.torque
        ))
    }
}

impl From<PyRobstrideMotorControlParams> for RobstrideMotorControlParams {
    fn from(params: PyRobstrideMotorControlParams) -> Self {
        RobstrideMotorControlParams {
            position: params.position,
            velocity: params.velocity,
            kp: params.kp,
            kd: params.kd,
            torque: params.torque,
        }
    }
}

impl From<RobstrideMotorControlParams> for PyRobstrideMotorControlParams {
    fn from(params: RobstrideMotorControlParams) -> Self {
        PyRobstrideMotorControlParams {
            position: params.position,
            velocity: params.velocity,
            kp: params.kp,
            kd: params.kd,
            torque: params.torque,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
struct PyRobstrideMotorsSupervisor {
    inner: RobstrideMotorsSupervisor,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideMotorsSupervisor {
    #[new]
    #[pyo3(signature = (port_name, motor_infos, verbose = false, target_update_rate = 50.0, zero_on_init = false))]
    fn new(
        port_name: String,
        motor_infos: HashMap<u8, String>,
        verbose: bool,
        target_update_rate: f64,
        zero_on_init: bool,
    ) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = robstride_motor_type_from_str(type_str.as_str())?;
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let controller = RobstrideMotorsSupervisor::new(
            &port_name,
            &motor_infos,
            verbose,
            target_update_rate,
            zero_on_init,
        )
        .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(PyRobstrideMotorsSupervisor { inner: controller })
    }

    fn set_all_params(&self, params: HashMap<u8, PyRobstrideMotorControlParams>) -> PyResult<()> {
        let params: HashMap<u8, RobstrideMotorControlParams> = params
            .into_iter()
            .map(|(k, v)| (k, RobstrideMotorControlParams::from(v)))
            .collect();
        self.inner.set_all_params(params);
        Ok(())
    }

    fn set_params(&self, motor_id: u8, params: PyRobstrideMotorControlParams) -> PyResult<()> {
        self.inner
            .set_params(motor_id, RobstrideMotorControlParams::from(params))
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_positions(&self, positions: HashMap<u8, f32>) -> PyResult<()> {
        self.inner
            .set_positions(positions)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_position(&self, motor_id: u8, position: f32) -> PyResult<f32> {
        self.inner
            .set_position(motor_id, position)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_position(&self, motor_id: u8) -> PyResult<f32> {
        self.inner
            .get_position(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_velocities(&self, velocities: HashMap<u8, f32>) -> PyResult<()> {
        self.inner
            .set_velocities(velocities)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_velocity(&self, motor_id: u8, velocity: f32) -> PyResult<f32> {
        self.inner
            .set_velocity(motor_id, velocity)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_velocity(&self, motor_id: u8) -> PyResult<f32> {
        self.inner
            .get_velocity(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_kp(&self, motor_id: u8, kp: f32) -> PyResult<f32> {
        self.inner
            .set_kp(motor_id, kp)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_kp(&self, motor_id: u8) -> PyResult<f32> {
        self.inner
            .get_kp(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_kd(&self, motor_id: u8, kd: f32) -> PyResult<f32> {
        self.inner
            .set_kd(motor_id, kd)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_kd(&self, motor_id: u8) -> PyResult<f32> {
        self.inner
            .get_kd(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_torque(&self, motor_id: u8, torque: f32) -> PyResult<f32> {
        self.inner
            .set_torque(motor_id, torque)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_torque(&self, motor_id: u8) -> PyResult<f32> {
        self.inner
            .get_torque(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_torque_limit(&self, motor_id: u8, torque_limit: f32) -> PyResult<f32> {
        self.inner
            .set_torque_limit(motor_id, torque_limit)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_speed_limit(&self, motor_id: u8, speed_limit: f32) -> PyResult<f32> {
        self.inner
            .set_speed_limit(motor_id, speed_limit)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn set_current_limit(&self, motor_id: u8, current_limit: f32) -> PyResult<f32> {
        self.inner
            .set_current_limit(motor_id, current_limit)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn add_motor_to_zero(&self, motor_id: u8) -> PyResult<()> {
        self.inner
            .add_motor_to_zero(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn get_latest_feedback(&self) -> HashMap<u8, PyRobstrideMotorFeedback> {
        self.inner
            .get_latest_feedback()
            .into_iter()
            .map(|(k, v)| (k, v.into()))
            .collect()
    }

    fn toggle_pause(&self) -> PyResult<()> {
        self.inner.toggle_pause();
        Ok(())
    }

    fn stop(&self) -> PyResult<()> {
        self.inner.stop();
        Ok(())
    }

    fn __repr__(&self) -> PyResult<String> {
        let motor_count = self.inner.get_latest_feedback().len();
        Ok(format!(
            "PyRobstrideMotorsSupervisor(motor_count={})",
            motor_count
        ))
    }

    #[getter]
    fn total_commands(&self) -> PyResult<u64> {
        Ok(self.inner.get_total_commands())
    }

    fn failed_commands_for(&self, motor_id: u8) -> PyResult<u64> {
        self.inner
            .get_failed_commands(motor_id)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn reset_command_counters(&self) -> PyResult<()> {
        self.inner.reset_command_counters();
        Ok(())
    }

    fn is_running(&self) -> PyResult<bool> {
        Ok(self.inner.is_running())
    }

    #[setter]
    fn max_update_rate(&self, rate: f64) -> PyResult<()> {
        self.inner.set_max_update_rate(rate);
        Ok(())
    }

    #[getter]
    fn actual_update_rate(&self) -> PyResult<f64> {
        Ok(self.inner.get_actual_update_rate())
    }

    fn toggle_serial(&self) -> PyResult<bool> {
        Ok(self.inner.toggle_serial())
    }

    #[getter]
    fn serial(&self) -> PyResult<bool> {
        Ok(self.inner.get_serial())
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct PyRobstrideMotorConfig {
    #[pyo3(get)]
    p_min: f32,
    #[pyo3(get)]
    p_max: f32,
    #[pyo3(get)]
    v_min: f32,
    #[pyo3(get)]
    v_max: f32,
    #[pyo3(get)]
    kp_min: f32,
    #[pyo3(get)]
    kp_max: f32,
    #[pyo3(get)]
    kd_min: f32,
    #[pyo3(get)]
    kd_max: f32,
    #[pyo3(get)]
    t_min: f32,
    #[pyo3(get)]
    t_max: f32,
    #[pyo3(get)]
    zero_on_init: bool,
    #[pyo3(get)]
    can_timeout_command: u16,
    #[pyo3(get)]
    can_timeout_factor: f32,
}

impl From<RobstrideMotorConfig> for PyRobstrideMotorConfig {
    fn from(config: RobstrideMotorConfig) -> Self {
        PyRobstrideMotorConfig {
            p_min: config.p_min,
            p_max: config.p_max,
            v_min: config.v_min,
            v_max: config.v_max,
            kp_min: config.kp_min,
            kp_max: config.kp_max,
            kd_min: config.kd_min,
            kd_max: config.kd_max,
            t_min: config.t_min,
            t_max: config.t_max,
            zero_on_init: config.zero_on_init,
            can_timeout_command: config.can_timeout_command,
            can_timeout_factor: config.can_timeout_factor,
        }
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Copy, Clone, Debug, Eq)]
struct PyRobstrideMotorType {
    value: u8,
}

impl Hash for PyRobstrideMotorType {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.value.hash(state);
    }
}

impl PartialEq for PyRobstrideMotorType {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl PyRobstrideMotorType {
    #[classattr]
    const TYPE00: Self = PyRobstrideMotorType { value: 0 };
    #[classattr]
    const TYPE01: Self = PyRobstrideMotorType { value: 1 };
    #[classattr]
    const TYPE02: Self = PyRobstrideMotorType { value: 2 };
    #[classattr]
    const TYPE03: Self = PyRobstrideMotorType { value: 3 };
    #[classattr]
    const TYPE04: Self = PyRobstrideMotorType { value: 4 };

    fn __repr__(&self) -> PyResult<String> {
        let type_name = match self.value {
            0 => "TYPE00",
            1 => "TYPE01",
            2 => "TYPE02",
            3 => "TYPE03",
            4 => "TYPE04",
            _ => "Unknown",
        };
        Ok(format!("PyRobstrideMotorType::{}", type_name))
    }

    #[staticmethod]
    fn from_str(s: &str) -> PyResult<Self> {
        let motor_type = robstride_motor_type_from_str(s)?;
        Ok(PyRobstrideMotorType::from(motor_type))
    }

    fn __hash__(&self) -> PyResult<isize> {
        Ok(self.value as isize)
    }

    fn __eq__(&self, other: &Bound<'_, PyAny>) -> PyResult<bool> {
        if let Ok(other) = other.extract::<PyRobstrideMotorType>() {
            Ok(self.value == other.value)
        } else {
            Ok(false)
        }
    }
}

impl From<RobstrideMotorType> for PyRobstrideMotorType {
    fn from(motor_type: RobstrideMotorType) -> Self {
        match motor_type {
            RobstrideMotorType::Type00 => PyRobstrideMotorType::TYPE00,
            RobstrideMotorType::Type01 => PyRobstrideMotorType::TYPE01,
            RobstrideMotorType::Type02 => PyRobstrideMotorType::TYPE02,
            RobstrideMotorType::Type03 => PyRobstrideMotorType::TYPE03,
            RobstrideMotorType::Type04 => PyRobstrideMotorType::TYPE04,
        }
    }
}

impl From<PyRobstrideMotorType> for RobstrideMotorType {
    fn from(py_motor_type: PyRobstrideMotorType) -> Self {
        match py_motor_type.value {
            0 => RobstrideMotorType::Type00,
            1 => RobstrideMotorType::Type01,
            2 => RobstrideMotorType::Type02,
            3 => RobstrideMotorType::Type03,
            4 => RobstrideMotorType::Type04,
            _ => RobstrideMotorType::Type04,
        }
    }
}

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(get_version, m)?)?;
    m.add_class::<PyRobstrideMotors>()?;
    m.add_class::<PyRobstrideMotorFeedback>()?;
    m.add_class::<PyRobstrideMotorsSupervisor>()?;
    m.add_class::<PyRobstrideMotorControlParams>()?;
    m.add_class::<PyRobstrideMotorConfig>()?;
    m.add_class::<PyRobstrideMotorType>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

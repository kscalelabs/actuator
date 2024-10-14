use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use robstride::{
    motor_type_from_str as robstride_motor_type_from_str,
    MotorControlParams as RobstrideMotorControlParams, MotorFeedback as RobstrideMotorFeedback,
    MotorType as RobstrideMotorType, Motors as RobstrideMotors,
    MotorsSupervisor as RobstrideMotorsSupervisor,
};
use std::collections::HashMap;

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

    fn send_set_zero(&mut self, motor_ids: Option<Vec<u8>>) -> PyResult<()> {
        self.inner
            .send_set_zeros(motor_ids.as_deref())
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
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
    #[pyo3(signature = (port_name, motor_infos, verbose = false, target_update_rate = 50.0))]
    fn new(
        port_name: String,
        motor_infos: HashMap<u8, String>,
        verbose: bool,
        target_update_rate: f64,
    ) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = robstride_motor_type_from_str(type_str.as_str())?;
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let controller =
            RobstrideMotorsSupervisor::new(&port_name, &motor_infos, verbose, target_update_rate)
                .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(PyRobstrideMotorsSupervisor { inner: controller })
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

    fn set_params(&self, motor_id: u8, params: &PyRobstrideMotorControlParams) -> PyResult<()> {
        self.inner.set_params(
            motor_id,
            RobstrideMotorControlParams {
                position: params.position,
                velocity: params.velocity,
                kp: params.kp,
                kd: params.kd,
                torque: params.torque,
            },
        );
        Ok(())
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

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyRobstrideMotors>()?;
    m.add_class::<PyRobstrideMotorFeedback>()?;
    m.add_class::<PyRobstrideMotorsSupervisor>()?;
    m.add_class::<PyRobstrideMotorControlParams>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

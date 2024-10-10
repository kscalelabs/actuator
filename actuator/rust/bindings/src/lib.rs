use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use robstride::{
    MotorFeedback as RobstrideMotorFeedback, MotorType as RobstrideMotorType,
    Motors as RobstrideMotors,
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
    fn new(port_name: String, motor_infos: HashMap<u8, String>) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = match type_str.as_str() {
                    "01" => RobstrideMotorType::Type01,
                    "03" => RobstrideMotorType::Type03,
                    "04" => RobstrideMotorType::Type04,
                    _ => {
                        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                            "Invalid motor type: {}",
                            type_str
                        )))
                    }
                };
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let motors = RobstrideMotors::new(&port_name, motor_infos)
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

    fn send_set_zero(&mut self) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        self.inner
            .send_set_zero()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn send_reset(&mut self) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        self.inner
            .send_reset()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn send_start(&mut self) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        self.inner
            .send_start()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn send_torque_controls(
        &mut self,
        torque_sets: HashMap<u8, f32>,
    ) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        self.inner
            .send_torque_controls(&torque_sets)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn get_latest_feedback(&self) -> HashMap<u8, PyRobstrideMotorFeedback> {
        self.inner
            .get_latest_feedback()
            .into_iter()
            .map(|(k, v)| (k, v.into()))
            .collect()
    }

    fn get_latest_feedback_for(&self, motor_id: u8) -> PyResult<PyRobstrideMotorFeedback> {
        self.inner
            .get_latest_feedback_for(motor_id)
            .map(|feedback| feedback.clone().into())
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }

    fn __repr__(&self) -> PyResult<String> {
        let motor_count = self.inner.get_latest_feedback().len();
        Ok(format!("PyRobstrideMotors(motor_count={})", motor_count))
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

#[pymodule]
fn bindings(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyRobstrideMotors>()?;
    m.add_class::<PyRobstrideMotorFeedback>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

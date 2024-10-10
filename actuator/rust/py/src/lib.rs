use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};
use robstride::{MotorFeedback, MotorInfo, MotorType, Motors};
use std::collections::HashMap;

#[gen_stub_pyclass]
#[pyclass]
struct PyMotors {
    inner: Motors,
}

#[gen_stub_pymethods]
#[pymethods]
impl PyMotors {
    #[new]
    fn new(port_name: String, motor_infos: Vec<(u8, String)>) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = match type_str.as_str() {
                    "Type01" => MotorType::Type01,
                    "Type03" => MotorType::Type03,
                    "Type04" => MotorType::Type04,
                    _ => {
                        return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(
                            "Invalid motor type",
                        ))
                    }
                };
                Ok(MotorInfo { id, motor_type })
            })
            .collect::<PyResult<Vec<MotorInfo>>>()?;

        let motors = Motors::new(&port_name, motor_infos)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(PyMotors { inner: motors })
    }

    fn send_get_mode(&mut self) -> PyResult<HashMap<u8, String>> {
        self.inner
            .send_get_mode()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, format!("{:?}", v))))
            .collect()
    }

    fn send_set_zero(&mut self) -> PyResult<HashMap<u8, PyMotorFeedback>> {
        self.inner
            .send_set_zero()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn send_reset(&mut self) -> PyResult<HashMap<u8, PyMotorFeedback>> {
        self.inner
            .send_reset()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn send_start(&mut self) -> PyResult<HashMap<u8, PyMotorFeedback>> {
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
    ) -> PyResult<HashMap<u8, PyMotorFeedback>> {
        self.inner
            .send_torque_controls(&torque_sets)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?
            .into_iter()
            .map(|(k, v)| Ok((k, v.into())))
            .collect()
    }

    fn get_latest_feedback(&self) -> HashMap<u8, PyMotorFeedback> {
        self.inner
            .get_latest_feedback()
            .into_iter()
            .map(|(k, v)| (k, v.into()))
            .collect()
    }

    fn get_latest_feedback_for(&self, motor_id: u8) -> PyResult<PyMotorFeedback> {
        self.inner
            .get_latest_feedback_for(motor_id)
            .map(|feedback| feedback.clone().into())
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))
    }
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
struct PyMotorFeedback {
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

impl From<MotorFeedback> for PyMotorFeedback {
    fn from(feedback: MotorFeedback) -> Self {
        PyMotorFeedback {
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
fn py(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<PyMotors>()?;
    m.add_class::<PyMotorFeedback>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

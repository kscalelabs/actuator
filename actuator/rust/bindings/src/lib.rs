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
use std::time::Duration;
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
                let motor_type = robstride_motor_type_from_str(type_str.as_str())?;
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let motors = RobstrideMotors::new(&port_name, &motor_infos)
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

    fn send_set_zero(
        &mut self,
        motor_ids: Option<Vec<u8>>,
    ) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        self.inner
            .send_set_zero(motor_ids.as_deref())
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

    fn send_motor_controls(
        &mut self,
        motor_controls: HashMap<u8, PyRobstrideMotorControlParams>,
    ) -> PyResult<HashMap<u8, PyRobstrideMotorFeedback>> {
        let motor_controls: HashMap<u8, RobstrideMotorControlParams> = motor_controls
            .into_iter()
            .map(|(k, v)| (k, v.into()))
            .collect();

        self.inner
            .send_motor_controls(&motor_controls)
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
    fn new(port_name: String, motor_infos: HashMap<u8, String>) -> PyResult<Self> {
        let motor_infos = motor_infos
            .into_iter()
            .map(|(id, type_str)| {
                let motor_type = robstride_motor_type_from_str(type_str.as_str())?;
                Ok((id, motor_type))
            })
            .collect::<PyResult<HashMap<u8, RobstrideMotorType>>>()?;

        let controller = RobstrideMotorsSupervisor::new(&port_name, &motor_infos)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;

        Ok(PyRobstrideMotorsSupervisor { inner: controller })
    }

    fn set_position(&self, motor_id: u8, position: f32) -> PyResult<()> {
        self.inner.set_position(motor_id, position);
        Ok(())
    }

    fn set_velocity(&self, motor_id: u8, velocity: f32) -> PyResult<()> {
        self.inner.set_velocity(motor_id, velocity);
        Ok(())
    }

    fn set_kp(&self, motor_id: u8, kp: f32) -> PyResult<()> {
        self.inner.set_kp(motor_id, kp);
        Ok(())
    }

    fn set_kd(&self, motor_id: u8, kd: f32) -> PyResult<()> {
        self.inner.set_kd(motor_id, kd);
        Ok(())
    }

    fn set_torque(&self, motor_id: u8, torque: f32) -> PyResult<()> {
        self.inner.set_torque(motor_id, torque);
        Ok(())
    }

    fn set_sleep_duration(&self, sleep_duration: f32) -> PyResult<()> {
        self.inner
            .set_sleep_duration(Duration::from_millis(sleep_duration as u64));
        Ok(())
    }

    fn add_motor_to_zero(&self, motor_id: u8) -> PyResult<()> {
        self.inner.add_motor_to_zero(motor_id);
        Ok(())
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

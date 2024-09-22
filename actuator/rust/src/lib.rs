use env_logger::Builder;
use log::{debug, LevelFilter};
use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::sync::Once;
use tokio::runtime::Runtime;

mod robstride;
use robstride::Robstride;

pub trait ActuatorImpl: Send + Sync {
    fn set_position(&self, actuator_id: u8, position: f64);
    fn get_position(&self, actuator_id: u8) -> f64;
    fn update(&self) -> std::pin::Pin<Box<dyn std::future::Future<Output = ()> + Send + '_>>;
    fn set_pid_params(
        &self,
        actuator_id: u8,
        pos_kp: f64,
        pos_kd: f64,
        vel_kp: f64,
        vel_kd: f64,
        tor_kp: f64,
        tor_kd: f64,
    );
    fn get_pid_params(&self, actuator_id: u8) -> (f64, f64, f64, f64, f64, f64);
}

#[pyclass]
#[derive(Clone, Copy)]
pub enum ActuatorType {
    Robstride,
}

#[pyclass]
#[derive(Clone)]
struct Actuator {
    actuator_impl: Arc<dyn ActuatorImpl>,
    #[allow(dead_code)]
    runtime: Arc<Runtime>,
    running: Arc<AtomicBool>,
}

static INIT: Once = Once::new();

#[pymethods]
impl Actuator {
    #[new]
    #[pyo3(signature = (actuator_type, debug_mode = false))]
    fn new(actuator_type: ActuatorType, debug_mode: bool) -> PyResult<Self> {
        INIT.call_once(|| {
            let log_level = if debug_mode {
                LevelFilter::Debug
            } else {
                LevelFilter::Info
            };
            Builder::new().filter_level(log_level).init();
            debug!("Logging initialized with level: {:?}", log_level);
        });

        let runtime = Runtime::new()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let runtime = Arc::new(runtime);
        let running = Arc::new(AtomicBool::new(true));

        let actuator_impl: Arc<dyn ActuatorImpl> = match actuator_type {
            ActuatorType::Robstride => Arc::new(Robstride::new(debug_mode)),
        };
        let actuator_impl_clone = Arc::clone(&actuator_impl);
        let running_clone = Arc::clone(&running);

        runtime.spawn(async move {
            while running_clone.load(Ordering::Relaxed) {
                actuator_impl_clone.update().await;
            }
        });

        Ok(Actuator {
            actuator_impl,
            runtime,
            running,
        })
    }

    fn set_position(&self, actuator_id: u8, position: f64) -> PyResult<()> {
        self.actuator_impl.set_position(actuator_id, position);
        Ok(())
    }

    fn get_position(&self, actuator_id: u8) -> PyResult<f64> {
        Ok(self.actuator_impl.get_position(actuator_id))
    }

    fn stop(&self) -> PyResult<()> {
        self.running.store(false, Ordering::Relaxed);
        Ok(())
    }

    fn set_pid_params(
        &self,
        actuator_id: u8,
        pos_kp: f64,
        pos_kd: f64,
        vel_kp: f64,
        vel_kd: f64,
        tor_kp: f64,
        tor_kd: f64,
    ) -> PyResult<()> {
        self.actuator_impl.set_pid_params(
            actuator_id,
            pos_kp,
            pos_kd,
            vel_kp,
            vel_kd,
            tor_kp,
            tor_kd,
        );
        Ok(())
    }

    fn get_pid_params(&self, actuator_id: u8) -> PyResult<(f64, f64, f64, f64, f64, f64)> {
        Ok(self.actuator_impl.get_pid_params(actuator_id))
    }
}

impl Drop for Actuator {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[pymodule]
fn lib(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_class::<Actuator>()?;
    m.add_class::<ActuatorType>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

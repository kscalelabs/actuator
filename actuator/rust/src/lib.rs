use pyo3::prelude::*;
use pyo3_stub_gen::{define_stub_info_gatherer, derive::gen_stub_pyclass};
use std::sync::{Arc, Mutex};
use tokio::runtime::Runtime;
use tokio::time::{interval, Duration};
use std::sync::atomic::{AtomicBool, Ordering};

#[pyclass]
#[gen_stub_pyclass]
struct Actuator {
    runtime: Arc<Mutex<Option<Runtime>>>,
    running: Arc<AtomicBool>,
}

#[pymethods]
impl Actuator {
    #[new]
    fn new() -> PyResult<Self> {
        let runtime = Runtime::new().map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(e.to_string()))?;
        let runtime = Arc::new(Mutex::new(Some(runtime)));
        let running = Arc::new(AtomicBool::new(true));

        let runtime_clone = Arc::clone(&runtime);
        let running_clone = Arc::clone(&running);
        std::thread::spawn(move || {
            if let Some(rt) = runtime_clone.lock().unwrap().as_ref() {
                rt.block_on(async {
                    let mut interval = interval(Duration::from_secs(1));
                    while running_clone.load(Ordering::Relaxed) {
                        interval.tick().await;
                        println!("Hello, world!");
                    }
                });
            }
        });

        Ok(Actuator { runtime, running })
    }

    fn stop(&mut self) -> PyResult<()> {
        self.running.store(false, Ordering::Relaxed);
        if let Some(runtime) = self.runtime.lock().unwrap().take() {
            runtime.shutdown_background();
        }
        Ok(())
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
    Ok(())
}

define_stub_info_gatherer!(stub_info);

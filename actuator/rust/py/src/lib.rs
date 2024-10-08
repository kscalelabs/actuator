use pyo3::prelude::*;
use pyo3_stub_gen::{define_stub_info_gatherer, derive::gen_stub_pyfunction};

#[gen_stub_pyfunction]
#[pyfunction]
fn hello_world() -> PyResult<()> {
    println!("Hello, world!");
    Ok(())
}

#[pymodule]
fn py(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello_world, m)?)?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

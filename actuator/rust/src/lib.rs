use pyo3::prelude::*;
use pyo3_stub_gen::{define_stub_info_gatherer, derive::gen_stub_pyfunction};

#[gen_stub_pyfunction]
#[pyfunction]
fn hello_world() {
    println!("Hello, world!");
}

#[gen_stub_pyfunction]
#[pyfunction]
fn sum(v: Vec<u32>) -> u32 {
    v.iter().sum()
}

#[pymodule]
fn lib(m: &Bound<PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(hello_world, m)?)?;
    m.add_function(wrap_pyfunction!(sum, m)?)?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

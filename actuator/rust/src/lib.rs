use pyo3::prelude::*;

#[pymodule]
fn lib(_py: Python, _m: &PyModule) -> PyResult<()> {
    println!("Hello, world!");
    Ok(())
}

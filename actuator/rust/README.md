# Rust

To build the Rust extension:

```bash
cargo build
```

To run the stub generator:

```bash
cargo run --bin stub_gen
```

On Linux, you may need to install libudev-dev for Rust to properly build.

```bash
sudo apt-get install libudev-dev
```

To run the profiling script:

```bash
cargo run --bin profile
```

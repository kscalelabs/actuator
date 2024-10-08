use robstride::{Motor, RunMode, ROBSTRIDE01_CONFIG};
use std::thread;
use std::time::{Duration, Instant};
use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use ctrlc;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting sinusoidal profiling");

    let mut motor = Motor::new("/dev/ttyCH341USB0", ROBSTRIDE01_CONFIG, 1)?;

    motor.send_set_mode(RunMode::PositionMode)?;
    thread::sleep(Duration::from_millis(50));

    motor.send_reset()?;
    motor.send_start()?;
    thread::sleep(Duration::from_millis(50));

    motor.send_set_speed_limit(10.0)?;
    thread::sleep(Duration::from_millis(50));

    let period = 5.0; // seconds
    let amplitude = PI / 2.0;
    let duration = 20.0; // total duration of the profiling in seconds
    let start_time = Instant::now();

    let mut file = File::create("motor_profile.csv")?;
    writeln!(file, "Step,Desired Position,Actual Position")?;

    let loop_duration = Duration::from_millis(4); // 250 Hz
    let mut step = 0;

    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    while running.load(Ordering::SeqCst) && start_time.elapsed().as_secs_f32() < duration {
        let loop_start = Instant::now();
        let elapsed = start_time.elapsed().as_secs_f32();
        let desired_position = amplitude * (2.0 * PI * elapsed / period).sin();
        let feedback = motor.send_set_location(desired_position)?;

        let actual_position = feedback.position;

        writeln!(file, "{},{},{}", step, desired_position, actual_position)?;

        let elapsed_loop = loop_start.elapsed();
        if elapsed_loop < loop_duration {
            thread::sleep(loop_duration - elapsed_loop);
        }

        step += 1;
    }

    motor.send_reset()?;
    println!("Sinusoidal profiling finished");
    Ok(())
}

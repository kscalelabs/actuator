use robstride::{Motor, RunMode, ROBSTRIDE_CONFIGS};
use std::thread;
use std::time::{Duration, Instant};
use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use ctrlc;

fn calculate_torque(desired_position: f32, actual_position: f32, kp: f32, kd: f32, actual_velocity: f32) -> f32 {
    let position_error = desired_position - actual_position;
    let velocity_error = -actual_velocity;
    kp * position_error + kd * velocity_error
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting sinusoidal profiling");

    // Use the updated MotorConfig from ROBSTRIDE_CONFIGS
    let motor_config = ROBSTRIDE_CONFIGS.get("01").expect("Config not found");
    let mut motor = Motor::new("/dev/ttyCH341USB0", motor_config, 1)?;

    motor.send_set_mode(RunMode::MitMode)?;
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

        // Read feedback from the motor
        let responses = motor.read_all_pending_responses()?;
        let feedback = responses.get(0).unwrap();

        let torque = calculate_torque(desired_position, feedback.position, 5.0, 0.1, feedback.velocity);

        writeln!(file, "{},{},{}", step, desired_position, feedback.position)?;
        println!("{},{},{},{}", step, desired_position, feedback.position, torque);

        motor.send_torque_control(torque)?;

        let elapsed_loop = loop_start.elapsed();
        if elapsed_loop < loop_duration {
            thread::sleep(loop_duration - elapsed_loop);
        }

        step += 1;
    }

    motor.send_reset()?;
    println!("Sinusoidal profiling finished. Closed cleanly.");
    Ok(())
}

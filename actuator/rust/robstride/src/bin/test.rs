use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};

use robstride::{MotorControlParams, MotorType, MotorsSupervisor, RunMode};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting Rust test script");

    // Define motor types and IDs
    let mut motor_infos = HashMap::new();
    motor_infos.insert(1, MotorType::Type03);
    motor_infos.insert(2, MotorType::Type03);
    motor_infos.insert(3, MotorType::Type03);
    motor_infos.insert(4, MotorType::Type03);
    motor_infos.insert(5, MotorType::Type03);

    // Initialize the MotorsSupervisor
    let supervisor = MotorsSupervisor::new("/dev/ttyCH341USB1", &motor_infos, false, 100.0, 200.0)?;

    // Sine wave control parameters
    let pi = std::f64::consts::PI;
    let magnitude = pi / 4.0;
    let period = 2.0;
    let duration = 10.0; // Run for 10 seconds
    let kp = 5.0;
    let kd = 0.1;

    let start_time = Instant::now();
    let mut elapsed_time = 0.0;

    for id in 1..=5 {
        supervisor.add_motor_to_zero(id)?;
    }

    for id in 1..=5 {
        supervisor.set_kp(id, kp)?;
        supervisor.set_kd(id, kd)?;
    }

    let mut counter = 0;

    while elapsed_time < duration {
        counter += 1;

        for id in 1..=5 {
            let desired_position = magnitude * (2.0 * pi * elapsed_time / period).sin();
            supervisor.set_position(id, desired_position as f32)?;
        }

        // Send control commands
        let feedbacks = supervisor.get_latest_feedback();

        // Print feedback
        for (id, feedback) in feedbacks {
            println!(
                "Motor {} - Time: {:.2}s, Current pos: {:.3}, Current vel: {:.3}",
                id, elapsed_time, feedback.position, feedback.velocity
            );
        }

        // Update elapsed time
        elapsed_time = start_time.elapsed().as_secs_f64();

        // Sleep to maintain update rate of 50 Hz
        let time_to_sleep = 0.02 - start_time.elapsed().as_secs_f64();
        if time_to_sleep > 0.0 {
            thread::sleep(Duration::from_secs_f64(time_to_sleep));
        }
    }

    supervisor.stop();

    println!("Program finished");
    println!("Average loop frequency: {} Hz", counter as f64 / elapsed_time);
    Ok(())
}

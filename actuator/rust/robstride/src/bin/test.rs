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
    let supervisor = MotorsSupervisor::new("/dev/ttyCH341USB1", &motor_infos, true, 100.0, 0.5)?;

    // Set all motors to MIT mode
    supervisor.send_set_mode(RunMode::MitMode)?;

    // Sine wave control parameters
    let pi = std::f64::consts::PI;
    let magnitude = pi / 4.0;
    let period = 2.0;
    let duration = 10.0; // Run for 10 seconds
    let kp = 5.0;
    let kd = 0.1;

    let start_time = Instant::now();
    let mut elapsed_time = 0.0;

    while elapsed_time < duration {
        let mut params_map = HashMap::new();

        for id in 1..=5 {
            let phase_offset = 2.0 * pi * (id as f64 - 1.0) / 5.0;
            let desired_position = magnitude * (2.0 * pi * elapsed_time / period + phase_offset).sin();

            params_map.insert(
                id,
                MotorControlParams {
                    position: desired_position as f32,
                    velocity: 0.0,
                    kp: kp as f32,
                    kd: kd as f32,
                    torque: 0.0,
                },
            );
        }

        // Send control commands
        let feedbacks = supervisor.send_motor_controls(&params_map, true)?;

        // Print feedback
        for (id, feedback) in feedbacks {
            println!(
                "Motor {} - Time: {:.2}s, Desired pos: {:.3}, Current pos: {:.3}, Current vel: {:.3}",
                id, elapsed_time, params_map[&id].position, feedback.position, feedback.velocity
            );
        }

        // Update elapsed time
        elapsed_time = start_time.elapsed().as_secs_f64();

        // Sleep to maintain update rate
        thread::sleep(Duration::from_millis(50));
    }

    // Reset all motors after the loop
    supervisor.send_resets()?;

    println!("Program finished");
    Ok(())
}

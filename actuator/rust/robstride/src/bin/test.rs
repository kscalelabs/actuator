use std::collections::HashMap;
use std::thread;
use std::time::{Duration, Instant};

use robstride::{MotorControlParams, MotorType, Motors, RunMode};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting Rust test script");

    // Define motor types and IDs
    let mut motor_infos = HashMap::new();
    motor_infos.insert(1, MotorType::Type04);
    motor_infos.insert(2, MotorType::Type04);
    motor_infos.insert(3, MotorType::Type04);
    motor_infos.insert(4, MotorType::Type01);
    motor_infos.insert(5, MotorType::Type01);

    let mut motors = Motors::new("/dev/ttyCH341USB1", &motor_infos, false)?;

    motors.send_resets()?;

    motors.send_starts()?;

    let duration = Duration::from_secs(10);

    let start_time = Instant::now();

    let mut pos_change = 0.1;
    let mut desired_position = 0.0;

    let mut loop_count = 0;

    // Initialize motor_controls hashmap with specified starting values for all motors 1-5
    let mut motor_controls = HashMap::from_iter((1..=5).map(|id| {
        let mut params = MotorControlParams::default();
        params.kp = 10.0;
        params.kd = 0.1;
        (id, params)
    }));

    while start_time.elapsed() < duration {
        loop_count += 1;

        let loop_start_time = Instant::now();

        // Sinsoidal motion
        desired_position += pos_change;

        if desired_position > 3.0 {
            pos_change = -0.1;
        } else if desired_position < -3.0 {
            pos_change = 0.1;
        }

        for (_, params) in &mut motor_controls {
            params.position = desired_position;
        }

        match motors.send_motor_controls(&motor_controls, true) {
            Ok(_) => (),
            Err(e) => println!("Failed to send motor controls: {:?}", e),
        }

        // // Enforce 50 Hz
        // let elapsed = loop_start_time.elapsed();
        // if elapsed > Duration::from_secs_f32(1.0 / 50.0) {
        //     println!("Elapsed: {:?}", elapsed);
        // } else {
        //     thread::sleep(Duration::from_secs_f32(1.0 / 50.0) - elapsed);
        // }
        thread::sleep(Duration::from_secs_f32(1.0 / 100.0));
    }

    println!("Loop count: {}", loop_count);
    println!(
        "Average frequency: {:?} Hz",
        loop_count as f32 / start_time.elapsed().as_secs_f32()
    );

    motors.send_resets()?;

    Ok(())
}

use ctrlc;
use robstride::{Motor, Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::f32::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Create motor instances
    let motor1 = Motor::new(&ROBSTRIDE_CONFIGS["04"], 1);
    let motor2 = Motor::new(&ROBSTRIDE_CONFIGS["01"], 2);

    // Insert motors into a HashMap
    let mut motors_map = HashMap::new();
    motors_map.insert(1, motor1);
    motors_map.insert(2, motor2);

    // Create a Motors instance with the port name
    let mut motors = Motors::new("/dev/ttyCH341USB0", motors_map)?; // Adjust the device path as needed

    motors.send_reset(1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_mode(1, RunMode::MitMode)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_start(1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_speed_limit(1, 10.0)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_zero(1)?;

    motors.read_all_pending_responses()?;

    motors.send_reset(2)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_mode(2, RunMode::MitMode)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_start(2)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_speed_limit(2, 10.0)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_zero(2)?;

    motors.read_all_pending_responses()?;

    let start_time = Instant::now();
    let mut command_count = 0; // Initialize a counter for commands

    // PD controller parameters
    let kp_04 = 4.0;
    let kd_04 = 0.1;
    let kp_01 = 1.0;
    let kd_01 = 0.1;

    // Define period and amplitude
    let period = 2.0;
    let amplitude = PI;

    while running.load(Ordering::SeqCst) && start_time.elapsed() < Duration::new(20, 0) {
        let elapsed_time = start_time.elapsed().as_secs_f32();

        // Calculate desired positions using a sinusoidal function with specified period and amplitude
        let desired_position_1 = amplitude * (elapsed_time * 2.0 * PI / period).sin();
        let desired_position_2 = -amplitude * (elapsed_time * 2.0 * PI / period).sin();

        // Get current feedback for each motor
        let current_position_1 = motors.get_latest_feedback(1).map_or(0.0, |f| f.position) as f32;
        let current_position_2 = motors.get_latest_feedback(2).map_or(0.0, |f| f.position) as f32;

        // Calculate velocity (derivative of position)
        let current_velocity_1 = motors.get_latest_feedback(1).map_or(0.0, |f| f.velocity) as f32;
        let current_velocity_2 = motors.get_latest_feedback(2).map_or(0.0, |f| f.velocity) as f32;

        // Calculate torque using PD control
        let torque_1 =
            kp_04 * (desired_position_1 - current_position_1) - kd_04 * current_velocity_1;
        let torque_2 =
            kp_01 * (desired_position_2 - current_position_2) - kd_01 * current_velocity_2;

        // Send torque commands to the motors
        motors.send_torque_control(1, torque_1 as f32)?;
        std::thread::sleep(Duration::from_millis(4)); // Sleep to prevent overwhelming the bus
        motors.send_torque_control(2, torque_2 as f32)?;
        std::thread::sleep(Duration::from_millis(4));

        // Increment the command counter
        command_count += 2; // Two commands sent per loop iteration

        // Read feedback from the motors
        motors.read_all_pending_responses()?;

        // Calculate and print the command rate
        let commands_per_second = command_count as f32 / elapsed_time;
        println!("Commands per second: {:.2}", commands_per_second);
    }

    let elapsed_time = start_time.elapsed().as_secs_f32();

    println!("Done");
    println!(
        "Average control frequency: {:.2} Hz",
        (command_count as f32 / elapsed_time) / 2.0
    ); // Divide by 2 because two commands are sent per loop

    // Reset motors on exit
    motors.send_reset(1)?;
    motors.send_reset(2)?;

    Ok(())
}

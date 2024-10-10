use robstride::{Motor, Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use ctrlc;

const TEST_ID_1: u8 = 1;
const TEST_ID_2: u8 = 2;
const TORQUE_LIMIT: f32 = 20.0;
fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Create motor instances
    let motor_1 = Motor::new(&ROBSTRIDE_CONFIGS["04"], TEST_ID_1);
    let motor_2 = Motor::new(&ROBSTRIDE_CONFIGS["03"], TEST_ID_2);

    // Insert motors into a HashMap
    let mut motors_map = HashMap::new();
    motors_map.insert(TEST_ID_1, motor_1);
    motors_map.insert(TEST_ID_2, motor_2);

    // Create a Motors instance with the port name
    let mut motors = Motors::new("/dev/ttyCH341USB0", motors_map)?; // Adjust the device path as needed

    std::thread::sleep(Duration::from_millis(50));
    motors.send_reset(TEST_ID_1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_zero(TEST_ID_1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_reset(TEST_ID_1)?;
    std::thread::sleep(Duration::from_millis(100));
    motors.send_set_mode(TEST_ID_1, RunMode::MitMode)?;
    std::thread::sleep(Duration::from_millis(100));
    motors.send_start(TEST_ID_1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_speed_limit(TEST_ID_1, 5.0)?;
    std::thread::sleep(Duration::from_millis(50));

    std::thread::sleep(Duration::from_millis(50));
    motors.send_reset(TEST_ID_2)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_zero(TEST_ID_2)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_reset(TEST_ID_2)?;
    std::thread::sleep(Duration::from_millis(100));
    motors.send_set_mode(TEST_ID_2, RunMode::MitMode)?;
    std::thread::sleep(Duration::from_millis(100));
    motors.send_start(TEST_ID_2)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_speed_limit(TEST_ID_2, 5.0)?;
    std::thread::sleep(Duration::from_millis(50));

    motors.read_all_pending_responses()?;

    let start_time = Instant::now();
    let mut command_count = 0; // Initialize a counter for commands

    // PD controller parameters
    let kp_04 = 45.0;
    let kd_04 = 2.0;
    let kp_01 = 20.0;
    let kd_01 = 0.3;

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        // Desired position is 0 to hold the motor
        let desired_position_1 = 0.0;
        let desired_position_2 = 0.0;

        // Get current feedback for each motor
        let current_position_1 = motors.get_latest_feedback(TEST_ID_1).map_or(0.0, |f| f.position) as f32;
        let current_position_2 = motors.get_latest_feedback(TEST_ID_2).map_or(0.0, |f| f.position) as f32;

        // Calculate velocity (derivative of position)
        let current_velocity_1 = motors.get_latest_feedback(TEST_ID_1).map_or(0.0, |f| f.velocity) as f32;
        let current_velocity_2 = motors.get_latest_feedback(TEST_ID_2).map_or(0.0, |f| f.velocity) as f32;

        // Calculate torque using PD control
        let torque_1 = kp_04 * (desired_position_1 - current_position_1) - kd_04 * current_velocity_1;
        let torque_2 = kp_01 * (desired_position_2 - current_position_2) - kd_01 * current_velocity_2;

        let torque_1 = torque_1.clamp(-1.0*TORQUE_LIMIT, TORQUE_LIMIT);
        let torque_2 = torque_2.clamp(-1.0*TORQUE_LIMIT, TORQUE_LIMIT);

        // Send torque commands to the motors
        motors.send_torque_control(TEST_ID_1, torque_1 as f32)?;
        std::thread::sleep(Duration::from_millis(4)); // Sleep to prevent overwhelming the bus
        motors.send_torque_control(TEST_ID_2, torque_2 as f32)?;
        std::thread::sleep(Duration::from_millis(4)); // Sleep to prevent overwhelming the bus

        // Increment the command counter
        command_count += 1; // One command sent per loop iteration

        // Read feedback from the motors
        motors.read_all_pending_responses()?;

        // Print the latest feedback for each motor
        if let Some(feedback) = motors.get_latest_feedback(TEST_ID_1) {
            println!("Motor 1 Feedback: {:?}", feedback);
        }
        if let Some(feedback) = motors.get_latest_feedback(TEST_ID_2) {
            println!("Motor 2 Feedback: {:?}", feedback);
        }

        // Calculate and print the command rate
        let elapsed_time = start_time.elapsed().as_secs_f32();
        let commands_per_second = command_count as f32 / elapsed_time;
        println!("Commands per second: {:.2}", commands_per_second);
    }

    let elapsed_time = start_time.elapsed().as_secs_f32();

    println!("Done");
    println!("Average control frequency: {:.2} Hz", (command_count as f32 / elapsed_time));

    // Reset motors on exit
    motors.send_reset(TEST_ID_1)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_reset(TEST_ID_2)?;

    Ok(())
}

use robstride::{Motors, MotorType, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::f32::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::thread;

use ctrlc;

fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    let mut found_motor_ids: Vec<u8> = Vec::new();

    
    let MOTORS_PER_ROBSTRIDE = 127;

    // Iterate through potential motor IDs to find the valid one.
    for motor_id in (0..=MOTORS_PER_ROBSTRIDE).rev() {
        // let motor_id = *motor_id;
        let mut motors = Motors::new(
            "/dev/ttyUSB0",
            HashMap::from([(motor_id, MotorType::Type04)]),
        )?;
        println!("motor_id: {}", motor_id);

        motors.send_reset()?;
        std::thread::sleep(Duration::from_millis(50));

        let feedback_map = motors.get_latest_feedback_for(motor_id)?.clone();
        println!("feedback_map: {:?}", feedback_map);
        // if let Some(feedback) = feedback_map.get(&motor_id) {
        //     println!("Found motor with ID: {}, Position: {}, Velocity: {}, Torque: {}", 
        //              feedback.can_id, feedback.position, feedback.velocity, feedback.torque);
        //     found_motor_ids.push(motor_id);
        // }
        // Optional: wait for a short period to prevent too many attempts at once
        std::thread::sleep(Duration::from_millis(30));
    }

    if !found_motor_ids.is_empty() {
        println!("Successfully found motor IDs: {:?}", found_motor_ids);
    } else {
        println!("No valid motor IDs found.");
    }

    Ok(())
}

// Script to freeze the legs on Stompy Pro (5 dof per leg). Each motor has a sequentially increasing ID from 1 to 5.

use robstride::{Motor, Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use ctrlc;

const SAFETY_TORQUE_LIMIT: f32 = 1.0;
const NUM_MOTORS: u8 = 2;
const INIT_RETRIES: u8 = 1;

fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Create motor instances for the first Motors group
    let mut motors_map_1 = HashMap::new();
    for id in 1..=NUM_MOTORS {
        let motor: Motor;
        if id == 2 {
            motor = Motor::new(&ROBSTRIDE_CONFIGS["01"], id);
        } else {
            motor = Motor::new(&ROBSTRIDE_CONFIGS["04"], id);
        }
        motors_map_1.insert(id, motor);
    }

    // // Create motor instances for the second Motors group
    // let mut motors_map_2 = HashMap::new();
    // for id in 1..=5 {
    //     let motor = Motor::new(&ROBSTRIDE_CONFIGS["01"], id);
    //     motors_map_2.insert(id, motor);
    // }

    // Create Motors instances with the port names
    let mut motors_1 = Motors::new("/dev/ttyCH341USB0", motors_map_1)?;
    // let mut motors_2 = Motors::new("/dev/ttyCH341USB1", motors_map_2)?;

    // Function to initialize motors
    let initialize_motors = |motors: &mut Motors| -> Result<(), Box<dyn Error>> {
        for id in 1..=NUM_MOTORS {
            for _ in 0..INIT_RETRIES {
                motors.send_reset(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            for _ in 0..INIT_RETRIES {
                motors.send_set_zero(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            for _ in 0..INIT_RETRIES {
                motors.send_reset(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            for _ in 0..INIT_RETRIES {
                motors.send_set_mode(id, RunMode::MitMode)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            for _ in 0..INIT_RETRIES {
                motors.send_start(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            for _ in 0..INIT_RETRIES {
                motors.send_set_speed_limit(id, 10.0)?;
                std::thread::sleep(Duration::from_millis(50));
            }

            // for _ in 0..3 {
            //     motors.send_set_location(id, 0.0)?;
            //     std::thread::sleep(Duration::from_millis(50));
            // }

        }
        motors.read_all_pending_responses()?;
        Ok(())
    };

    let pd_control = |motors: &mut Motors, target_position: f32, kp_map: &HashMap<u8, f32>, kd_map: &HashMap<u8, f32>| -> Result<(), Box<dyn Error>> {
        for id in 1..=NUM_MOTORS {
            if let Some(feedback) = motors.get_latest_feedback(id) {
                let position_error = target_position - feedback.position;
                let velocity_error = -feedback.velocity;

                let kp = kp_map.get(&id).unwrap_or(&0.0);
                let kd = kd_map.get(&id).unwrap_or(&0.0);

                let torque_set = kp * position_error + kd * velocity_error;

                let torque_set = torque_set.clamp(-SAFETY_TORQUE_LIMIT, SAFETY_TORQUE_LIMIT);

                print!("Motor {} position: {} torque set: {}", id, feedback.position, torque_set);
                motors.send_torque_control(id, torque_set)?;
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        println!();
        motors.read_all_pending_responses()?;
        Ok(())
    };

    // Initialize both groups of motors
    std::thread::sleep(Duration::from_millis(100));
    initialize_motors(&mut motors_1)?;
    // std::thread::sleep(Duration::from_millis(100));
    // initialize_motors(&mut motors_2)?;

    println!("Motors initialized and set to stand position.");

    // Define PD constants for each motor
    let kp_map_1 = HashMap::from([
        (1, 5.0),
        (2, 5.0),
        (3, 5.0),
        (4, 5.0),
        (5, 5.0),
    ]);
    let kd_map_1 = HashMap::from([
        (1, 0.5),
        (2, 0.5),
        (3, 0.5),
        (4, 0.5),
        (5, 0.5),
    ]);
    let kp_map_2 = HashMap::from([
        (1, 1.0),
        (2, 1.0),
        (3, 1.0),
        (4, 1.0),
        (5, 1.0),
    ]);
    let kd_map_2 = HashMap::from([
        (1, 0.1),
        (2, 0.1),
        (3, 0.1),
        (4, 0.1),
        (5, 0.1),
    ]);

    // Wait until interrupted
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(50));
        pd_control(&mut motors_1, 0.0, &kp_map_1, &kd_map_1)?;
        std::thread::sleep(Duration::from_millis(50));
        motors_1.read_all_pending_responses()?;
        // for id in 1..=NUM_MOTORS {
        //     print!("Motor {} position: {}\t", id, motors_1.get_latest_feedback(id).unwrap().position);
        // }
        // println!();
        // pd_control(&mut motors_2, 0.0, &kp_map_2, &kd_map_2)?;
    }

    // Reset motors on exit
    for id in 1..=NUM_MOTORS {
        motors_1.send_reset(id)?;
        // motors_2.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(10));
    }

    println!("Motors reset. Exiting.");

    Ok(())
}

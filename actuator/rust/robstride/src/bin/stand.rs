// Script to freeze the legs on Stompy Pro (5 dof per leg). Each motor has a sequentially increasing ID from 1 to 5.

use robstride::{Motor, Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use std::time::Instant;
use ctrlc;

const SAFETY_TORQUE_LIMIT: f32 = 1.0;
const NUM_MOTORS: u8 = 5;
const INIT_RETRIES: u8 = 1;
const TORQUE_LIMIT: f32 = 20.0;

fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Create motor instances with specified configurations
    let mut motors_map = HashMap::new();
    let configs = ["04", "03", "03", "04", "01"];
    for id in 1..=NUM_MOTORS {
        let motor = Motor::new(&ROBSTRIDE_CONFIGS[configs[(id - 1) as usize]], id);
        motors_map.insert(id, motor);
    }

    // Create a Motors instance with the port name
    let mut motors = Motors::new("/dev/ttyCH341USB0", motors_map)?;

    // Initialize each motor
    for id in 1..=NUM_MOTORS {
        std::thread::sleep(Duration::from_millis(50));
        motors.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(50));
        if id != 5 {    
            motors.send_set_zero(id)?;
            std::thread::sleep(Duration::from_millis(50));
        }
        motors.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(100));
        motors.send_set_mode(id, RunMode::MitMode)?;
        std::thread::sleep(Duration::from_millis(100));
        motors.send_start(id)?;
        std::thread::sleep(Duration::from_millis(50));
        motors.send_set_speed_limit(id, 5.0)?;
        std::thread::sleep(Duration::from_millis(50));
    }

    motors.read_all_pending_responses()?;

    let start_time = Instant::now();
    let mut command_count = 0;

    // PD controller parameters
    let kp_map = HashMap::from([
        (1, 45.0),
        (2, 40.0),
        (3, 40.0),
        (4, 45.0),
        (5, 20.0),
    ]);
    let kd_map = HashMap::from([
        (1, 2.0),
        (2, 0.3),
        (3, 0.3),
        (4, 2.0),
        (5, 0.3),
    ]);

    std::thread::sleep(Duration::from_millis(100));

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        for id in 1..=NUM_MOTORS {
            let desired_position = 0.0;

            let current_position = motors.get_latest_feedback(id).map_or(0.0, |f| f.position) as f32;
            let current_velocity = motors.get_latest_feedback(id).map_or(0.0, |f| f.velocity) as f32;

            let kp = kp_map.get(&id).unwrap_or(&0.0);
            let kd = kd_map.get(&id).unwrap_or(&0.0);

            let torque = kp * (desired_position - current_position) - kd * current_velocity;
            let torque = torque.clamp(-TORQUE_LIMIT, TORQUE_LIMIT);

            motors.send_torque_control(id, torque)?;
            std::thread::sleep(Duration::from_millis(50));
        }

        command_count += 1;
        motors.read_all_pending_responses()?;

        let elapsed_time = start_time.elapsed().as_secs_f32();
        let commands_per_second = command_count as f32 / elapsed_time;
        println!("Commands per second: {:.2}", commands_per_second);
    }

    let elapsed_time = start_time.elapsed().as_secs_f32();
    println!("Done");
    println!("Average control frequency: {:.2} Hz", (command_count as f32 / elapsed_time));

    // Reset motors on exit
    for id in 1..=NUM_MOTORS {
        motors.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(50));
    }

    println!("Motors reset. Exiting.");

    Ok(())
}

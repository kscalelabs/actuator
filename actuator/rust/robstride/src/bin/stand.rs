use robstride::{Motor, Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::collections::HashMap;
use std::error::Error;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use ctrlc;

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
    for id in 1..=5 {
        let motor = Motor::new(&ROBSTRIDE_CONFIGS["04"], id);
        motors_map_1.insert(id, motor);
    }

    // Create motor instances for the second Motors group
    let mut motors_map_2 = HashMap::new();
    for id in 1..=5 {
        let motor = Motor::new(&ROBSTRIDE_CONFIGS["01"], id);
        motors_map_2.insert(id, motor);
    }

    // Create Motors instances with the port names
    let mut motors_1 = Motors::new("/dev/ttyCH341USB0", motors_map_1)?;
    let mut motors_2 = Motors::new("/dev/ttyCH341USB1", motors_map_2)?;

    // Function to initialize motors
    let initialize_motors = |motors: &mut Motors| -> Result<(), Box<dyn Error>> {
        for id in 1..=5 {
            motors.send_reset(id)?;
            // std::thread::sleep(Duration::from_millis(50));
            // motors.send_set_mode(id, RunMode::MitMode)?;
            std::thread::sleep(Duration::from_millis(50));
            motors.send_start(id)?;
            std::thread::sleep(Duration::from_millis(50));
            motors.send_set_zero(id)?;
            std::thread::sleep(Duration::from_millis(50));
            motors.send_set_location(id, 0.0)?; // Force set location to 0
        }
        motors.read_all_pending_responses()?;
        Ok(())
    };

    let set_location = |motors: &mut Motors, location: f32| -> Result<(), Box<dyn Error>> {
        for id in 1..=5 {
            motors.send_set_location(id, location)?;
            std::thread::sleep(Duration::from_millis(50));
        }
        motors.read_all_pending_responses()?;
        Ok(())
    };

    // Initialize both groups of motors
    initialize_motors(&mut motors_1)?;
    initialize_motors(&mut motors_2)?;

    println!("Motors initialized and set to stand position.");

    // Wait until interrupted
    while running.load(Ordering::SeqCst) {
        std::thread::sleep(Duration::from_millis(50));
        set_location(&mut motors_1, 0.0)?;
        set_location(&mut motors_2, 0.0)?;
    }

    // Reset motors on exit
    for id in 1..=5 {
        motors_1.send_reset(id)?;
        motors_2.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(50));
    }

    println!("Motors reset. Exiting.");

    Ok(())
}

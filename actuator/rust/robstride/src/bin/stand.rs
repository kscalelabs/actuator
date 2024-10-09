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
    std::thread::sleep(Duration::from_millis(100));
    let mut motors_2 = Motors::new("/dev/ttyCH341USB1", motors_map_2)?;
    std::thread::sleep(Duration::from_millis(100));
    // Function to initialize motors
    let initialize_motors = |motors: &mut Motors| -> Result<(), Box<dyn Error>> {
        for id in 1..=5 {
            for _ in 0..3 {
                motors.send_reset(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }
            std::thread::sleep(Duration::from_millis(100));

            for _ in 0..3 {
                motors.send_start(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }
            std::thread::sleep(Duration::from_millis(100));

            for _ in 0..3 {
                motors.send_set_zero(id)?;
                std::thread::sleep(Duration::from_millis(50));
            }
            std::thread::sleep(Duration::from_millis(100));

            for _ in 0..3 {
                motors.send_set_location(id, 0.0)?;
                std::thread::sleep(Duration::from_millis(50));
            }
        }
        motors.read_all_pending_responses()?;
        Ok(())
    };

    let set_location = |motors: &mut Motors, location: f32, kp: f32, kd: f32| -> Result<(), Box<dyn Error>> {
        for id in 1..=5 {
            retry_until_ok(|| motors.send_position_control(id, location, kp, kd).map_err(|e| Box::new(e) as Box<dyn Error>))?;
            std::thread::sleep(Duration::from_millis(100));
        }
        motors.read_all_pending_responses()?;
        Ok(())
    };

    // Initialize both groups of motors
    std::thread::sleep(Duration::from_millis(100));
    initialize_motors(&mut motors_1)?;
    std::thread::sleep(Duration::from_millis(100));
    initialize_motors(&mut motors_2)?;

    println!("Motors initialized and set to stand position.");

    set_location(&mut motors_1, 0.0, 180.0, 2.0)?;
    set_location(&mut motors_2, 0.0, 50.0, 0.4)?;
    // Wait until interrupted
    while running.load(Ordering::SeqCst) {
    //     std::thread::sleep(Duration::from_millis(100));
    //     set_location(&mut motors_1, 0.0, 60.0)?;
        std::thread::sleep(Duration::from_millis(100));
    //     set_location(&mut motors_2, 0.0, 30.0)?;
    }

    // Reset motors on exit
    for id in 1..=5 {
        motors_1.send_reset(id)?;
        motors_2.send_reset(id)?;
        std::thread::sleep(Duration::from_millis(100));
    }

    println!("Motors reset. Exiting.");

    Ok(())
}

fn retry_until_ok<F>(mut operation: F) -> Result<(), Box<dyn Error>>
where
    F: FnMut() -> Result<(), Box<dyn Error>>,
{
    let max_retries = 5;
    let mut attempts = 0;

    loop {
        match operation() {
            Ok(_) => return Ok(()),
            Err(e) => {
                println!("Error: {}", e);
                attempts += 1;
                if attempts >= max_retries {
                    return Err(e);
                }
                std::thread::sleep(Duration::from_millis(100)); // Wait before retrying
            }
        }
    }
}

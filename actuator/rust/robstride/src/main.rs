use ctrlc;
use robstride::{MotorInfo, MotorType, Motors, RunMode};
use std::error::Error;
use std::f32::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

const TEST_ID: u8 = 2;
const MAX_SECONDS: u64 = 5;
const MAX_TORQUE: f32 = 1.0;

fn main() -> Result<(), Box<dyn Error>> {
    // Create an atomic flag to handle SIGINT
    let running = Arc::new(AtomicBool::new(true));
    let r: Arc<AtomicBool> = running.clone();

    // Set up the SIGINT handler
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    // Create motor instances
    let mut motors = Motors::new(
        "/dev/ttyUSB0",
        vec![MotorInfo {
            id: TEST_ID,
            motor_type: MotorType::Type01,
        }],
    )?; // Adjust the device path as needed

    motors.send_reset(TEST_ID)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_mode(TEST_ID, RunMode::MitMode)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_start(TEST_ID)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_speed_limit(TEST_ID, 10.0)?;
    std::thread::sleep(Duration::from_millis(50));
    motors.send_set_zero(TEST_ID)?;

    motors.read_all_pending_responses()?;

    let start_time = Instant::now();
    let mut command_count = 0; // Initialize a counter for commands

    // PD controller parameters
    let kp_04 = 0.5;
    let kd_04 = 0.1;

    // Define period and amplitude
    let period = 1.0;
    let amplitude = PI / 2.0;

    while running.load(Ordering::SeqCst) && start_time.elapsed() < Duration::new(MAX_SECONDS, 0) {
        let elapsed_time = start_time.elapsed().as_secs_f32();

        let desired_position = amplitude * (elapsed_time * 2.0 * PI / period).sin();
        let current_position = motors
            .get_latest_feedback(TEST_ID)
            .map_or(0.0, |f| f.position) as f32;
        let current_velocity = motors
            .get_latest_feedback(TEST_ID)
            .map_or(0.0, |f| f.velocity) as f32;
        let torque = (kp_04 * (desired_position - current_position) - kd_04 * current_velocity)
            .clamp(-MAX_TORQUE, MAX_TORQUE);
        motors.send_torque_control(TEST_ID, torque as f32)?;
        std::thread::sleep(Duration::from_millis(4)); // Sleep to prevent overwhelming the bus

        command_count += 1;
        motors.read_all_pending_responses()?;

        if let Some(feedback) = motors.get_latest_feedback(TEST_ID) {
            println!("Motor 1 Feedback: {:?}", feedback);
        }

        let commands_per_second = command_count as f32 / elapsed_time;
        println!("Commands per second: {:.2}", commands_per_second);
    }

    let elapsed_time = start_time.elapsed().as_secs_f32();

    println!("Done");
    println!(
        "Average control frequency: {:.2} Hz",
        (command_count as f32 / elapsed_time)
    );

    motors.send_reset(TEST_ID)?;

    Ok(())
}

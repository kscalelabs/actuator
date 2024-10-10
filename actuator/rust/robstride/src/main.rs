use robstride::{MotorInfo, MotorType, Motors};
use std::error::Error;
use std::f32::consts::PI;
use std::time::Instant;

const TEST_ID: u8 = 2;
const RUN_TIME: f32 = 3.0;
const MAX_TORQUE: f32 = 1.0;

fn main() -> Result<(), Box<dyn Error>> {
    // Create motor instances
    let mut motors = Motors::new(
        "/dev/ttyUSB0",
        vec![MotorInfo {
            id: TEST_ID,
            motor_type: MotorType::Type01,
        }],
    )?;

    motors.send_reset()?;
    motors.send_set_zero()?;
    motors.send_start()?;

    let start_time = Instant::now();
    let mut command_count = 0;

    // PD controller parameters
    let kp_04 = 0.5;
    let kd_04 = 0.1;

    // Define period and amplitude
    let period = RUN_TIME;
    let amplitude = PI / 1.0;

    while start_time.elapsed().as_secs_f32() < RUN_TIME {
        // Track a sinusoidal trajectory over time.
        let elapsed_time = start_time.elapsed().as_secs_f32();
        let desired_position = amplitude * (elapsed_time * PI * 2.0 / period + PI / 2.0).cos();

        let feedback = motors.get_latest_feedback_for(TEST_ID)?.clone();

        let current_position = feedback.position;
        let current_velocity = feedback.velocity;
        let torque = (kp_04 * (desired_position - current_position) - kd_04 * current_velocity)
            .clamp(-MAX_TORQUE, MAX_TORQUE);

        motors.send_torque_controls(torque as f32)?;

        command_count += 1;
        println!(
            "Motor {} Commands: {}, Frequency: {:.2} Hz, Desired position: {:.2} Feedback: {:?}",
            TEST_ID,
            command_count,
            command_count as f32 / elapsed_time,
            desired_position,
            feedback
        );
    }

    motors.send_reset()?;

    let elapsed_time = start_time.elapsed().as_secs_f32();
    println!(
        "Done. Average control frequency: {:.2} Hz",
        (command_count as f32 / elapsed_time)
    );

    Ok(())
}

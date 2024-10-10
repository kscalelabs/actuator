use robstride::{motor_type_from_str, Motors};
use std::collections::HashMap;
use std::error::Error;
use std::f32::consts::PI;
use std::io::{self, Write};
use std::time::Instant;

const RUN_TIME: f32 = 3.0;
const MAX_TORQUE: f32 = 1.0;

fn run_motion_test(motors: &mut Motors, test_id: u8) -> Result<(), Box<dyn Error>> {
    motors.send_reset()?;
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

        let feedback = motors.get_latest_feedback_for(test_id)?.clone();
        let current_position = feedback.position;
        let current_velocity = feedback.velocity;
        let torque = (kp_04 * (desired_position - current_position) - kd_04 * current_velocity)
            .clamp(-MAX_TORQUE, MAX_TORQUE);

        motors.send_torque_controls(&HashMap::from([(test_id, torque)]))?;

        command_count += 1;
        println!(
            "Motor {} Commands: {}, Frequency: {:.2} Hz, Desired position: {:.2} Feedback: {:?}",
            test_id,
            command_count,
            command_count as f32 / elapsed_time,
            desired_position,
            feedback
        );
    }

    motors.send_torque_controls(&HashMap::from([(test_id, 0.0)]))?;
    motors.send_reset()?;

    let elapsed_time = start_time.elapsed().as_secs_f32();
    println!(
        "Done. Average control frequency: {:.2} Hz",
        (command_count as f32 / elapsed_time)
    );

    Ok(())
}

fn print_current_mode(motors: &mut Motors) {
    let modes = motors.send_get_mode();
    println!("Current mode: {:?}", modes.unwrap());
}

fn main() -> Result<(), Box<dyn Error>> {
    print!("Enter the TEST_ID (u8): ");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let test_id: u8 = input.trim().parse()?;

    print!("Enter the port name (default: /dev/ttyUSB0): ");
    io::stdout().flush()?;
    let mut port_input = String::new();
    io::stdin().read_line(&mut port_input)?;
    let port_name = port_input.trim().to_string();
    let port_name = if port_name.is_empty() {
        String::from("/dev/ttyUSB0")
    } else {
        port_name
    };

    print!("Enter the motor type (default: 01): ");
    io::stdout().flush()?;
    let mut motor_type_input = String::new();
    io::stdin().read_line(&mut motor_type_input)?;
    let motor_type_input = motor_type_input.trim().to_string();
    let motor_type_input = if motor_type_input.is_empty() {
        String::from("01")
    } else {
        motor_type_input
    };
    let motor_type = motor_type_from_str(motor_type_input.as_str())?;

    // Create motor instances
    let mut motors = Motors::new(&port_name, &HashMap::from([(test_id, motor_type)]))?;

    let mut last_command: i32 = -1;

    loop {
        println!("\nChoose an option:");
        println!("1. Print current mode");
        println!("2. Run motion test");
        println!("3. Exit");
        print!("Enter your choice (1-3), or press Enter to repeat the last command: ");
        io::stdout().flush()?;

        let mut choice = String::new();
        io::stdin().read_line(&mut choice)?;

        let choice = choice.trim();
        let command = if choice.is_empty() {
            last_command
        } else {
            choice.parse::<i32>().unwrap_or(-1)
        };

        match command {
            1 => {
                print_current_mode(&mut motors);
                last_command = 1;
            }
            2 => {
                run_motion_test(&mut motors, test_id)?;
                last_command = 2;
            }
            3 => break,
            -1 if choice.is_empty() => {
                println!("No previous command to repeat.");
            }
            _ => println!("Invalid choice. Please try again."),
        }
    }

    println!("Exiting program.");
    Ok(())
}

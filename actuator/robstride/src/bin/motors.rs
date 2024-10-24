use clap::Parser;
use robstride::{motor_type_from_str, MotorControlParams, MotorType, Motors};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::io::{self, Write};
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, help = "Enable verbose output")]
    verbose: bool,
}

fn sinusoid(
    motors: &mut Motors,
    id: u8,
    amplitude: f32,
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Running sinusoid test for {:?} with amplitude {:?}",
        duration, amplitude
    );

    motors.send_resets()?;
    motors.send_starts()?;
    motors.zero_motors(&[id])?;

    let start = Instant::now();
    let mut command_count = 0;
    let mut last_second = start;
    while start.elapsed() < duration {
        let t = start.elapsed().as_secs_f32();
        let pos = amplitude * (2.0 * PI * t).sin();
        motors.send_motor_controls(
            &HashMap::from([(
                id,
                MotorControlParams {
                    position: pos,
                    velocity: 0.0,
                    kp: 10.0,
                    kd: 1.0,
                    torque: 0.0,
                },
            )]),
            true,
        )?;

        command_count += 1;

        // Check if a second has passed
        if last_second.elapsed() > Duration::from_secs(1) {
            println!(
                "Commands per second: {}",
                command_count as f32 / start.elapsed().as_secs_f32()
            );
            last_second = Instant::now();
        }
    }

    let _ = motors.send_resets();

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

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
    let motor_infos: HashMap<u8, MotorType> = HashMap::from([(test_id, motor_type)]);
    let mut motors = Motors::new(&port_name, &motor_infos, args.verbose)?;

    println!("Motor Controller Test CLI");
    println!("Available commands:");
    println!("  sinusoid / s (<duration>)");
    println!("  zero / z");
    println!("  quit / q");

    loop {
        print!("> ");
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let parts: Vec<&str> = input.trim().split_whitespace().collect();

        if parts.is_empty() {
            continue;
        }

        match parts[0] {
            "sinusoid" | "s" => {
                let duration = Duration::from_secs(if parts.len() == 2 {
                    parts[1].parse::<u64>()?
                } else {
                    1
                });
                sinusoid(&mut motors, test_id, 1.0, duration)?;
                println!("Ran motor {} sinusoid test", test_id);
            }
            "zero" | "z" => {
                let _ = motors.zero_motors(&[test_id]);
                println!("Added motor {} to zero list", test_id);
            }
            "quit" | "q" => {
                println!("Exiting...");
                break;
            }
            _ => {
                println!("Unknown command");
            }
        }
    }

    Ok(())
}

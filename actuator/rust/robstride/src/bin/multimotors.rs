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
    ids: Vec<u8>,
    amplitude: f32,
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Running sinusoid test for {:?} with amplitude {:?}",
        duration, amplitude
    );

    motors.send_resets()?;
    motors.send_starts()?;

    let start = Instant::now();
    let mut command_count = 0;
    let mut last_second = start.clone();
    while start.elapsed() < duration {
        let t = start.elapsed().as_secs_f32();
        let pos = amplitude * (2.0 * PI * t).sin();
        motors.send_motor_controls(
            &HashMap::from(
                ids.iter()
                    .map(|id| {
                        (
                            *id,
                            MotorControlParams {
                                position: pos,
                                velocity: 0.0,
                                kp: 10.0,
                                kd: 1.0,
                                torque: 0.0,
                            },
                        )
                    })
                    .collect::<HashMap<u8, MotorControlParams>>(),
            ),
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

    print!("Enter the TEST_IDS (u8, ...): ");
    io::stdout().flush()?;
    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let test_ids: Vec<u8> = input
        .trim()
        .split(',')
        .map(|s| s.trim().parse().expect("Invalid ID"))
        .collect();

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

    print!("Enter the motor types (default: 01, ...): ");
    io::stdout().flush()?;
    let mut motor_types_input = String::new();
    io::stdin().read_line(&mut motor_types_input)?;
    let motor_types_input: HashMap<u8, String> = if motor_types_input.is_empty() {
        test_ids.iter().map(|id| (*id, "01".to_string())).collect()
    } else {
        let motor_types_input_vec: Vec<String> = motor_types_input
            .trim()
            .split(',')
            .map(|s| s.trim().to_string())
            .collect();
        if motor_types_input_vec.len() != test_ids.len() {
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                "Number of motor types must match number of test IDs",
            )));
        }
        test_ids
            .iter()
            .zip(motor_types_input_vec.iter())
            .map(|(id, motor_type)| (*id, motor_type.to_string()))
            .collect()
    };

    let motor_types: HashMap<u8, MotorType> = HashMap::from(
        test_ids
            .iter()
            .map(|id| {
                (
                    *id,
                    motor_type_from_str(motor_types_input[id].as_str())
                        .expect("Invalid motor type"),
                )
            })
            .collect::<HashMap<u8, MotorType>>(),
    );
    let mut motors = Motors::new(&port_name, &motor_types, args.verbose)?;

    println!("Motor Controller Test CLI");
    println!("Available commands:");
    println!("  sinusoid / s (<duration>)");
    println!("  clear-can / c");
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
                let _ = sinusoid(&mut motors, test_ids.clone(), 1.0, duration);
                println!("Ran motor {:?} sinusoid test", test_ids);
            }
            "clear-can" | "c" => {
                let _ = motors.send_can_timeout(0.0);
                println!("Cleared CAN timeout for {:?}", test_ids);
            }
            "zero" | "z" => {
                let _ = motors.zero_motors(&test_ids);
                println!("Added motor {:?} to zero list", test_ids);
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

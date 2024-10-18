use clap::Parser;
use robstride::{motor_type_from_str, MotorType, MotorsSupervisor};
use std::collections::HashMap;
use std::f32::consts::PI;
use std::io::{self, Write};
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, help = "Enable verbose output")]
    verbose: bool,
    #[arg(long, help = "Minimum update rate (Hz)", default_value_t = 10.0)]
    min_update_rate: f64,
    #[arg(long, help = "Maximum update rate (Hz)", default_value_t = 1000.0)]
    max_update_rate: f64,
    #[arg(long, help = "Zero on init", default_value_t = false)]
    zero_on_init: bool,
}

fn sinusoid(
    controller: &MotorsSupervisor,
    ids: &[u8],
    amplitude: f32,
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    println!(
        "Running sinusoid test for {:?} with amplitude {:?}",
        duration, amplitude
    );

    for &id in ids {
        controller.set_kd(id, 1.0)?;
        controller.set_kp(id, 10.0)?;
        controller.set_velocity(id, 0.0)?;
        controller.set_torque(id, 0.0)?;
    }

    let start = Instant::now();
    let mut last_second = start;
    controller.reset_command_counters();

    while start.elapsed() < duration {
        let t = start.elapsed().as_secs_f32();
        let pos = amplitude * (2.0 * PI * t).sin();
        for &id in ids {
            controller.set_position(id, pos)?;
        }
        std::thread::sleep(Duration::from_millis(10));

        // Check if a second has passed
        let total_commands = controller.get_total_commands();
        if last_second.elapsed() > Duration::from_secs(1) {
            println!(
                "Commands per second: {}",
                total_commands as f32 / start.elapsed().as_secs_f32()
            );
            last_second = Instant::now();
        }
    }

    for &id in ids {
        controller.set_position(id, 0.0)?;
    }

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
    let motor_types_input: HashMap<u8, String> = if motor_types_input.trim().is_empty() {
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

    let motor_types: HashMap<u8, MotorType> = motor_types_input
        .into_iter()
        .map(|(id, type_str)| {
            (
                id,
                motor_type_from_str(&type_str).expect("Invalid motor type"),
            )
        })
        .collect();

    let controller = MotorsSupervisor::new(
        &port_name,
        &motor_types,
        args.verbose,
        args.max_update_rate,
        args.zero_on_init,
    )?;

    println!("Motor Controller Test CLI");
    println!("Available commands:");
    println!("  p <position>");
    println!("  v <velocity>");
    println!("  t <torque>");
    println!("  kp <kp>");
    println!("  kd <kd>");
    println!("  sinusoid / s (<duration>)");
    println!("  zero / z");
    println!("  get_feedback / g");
    println!("  pause / w");
    println!("  reset / r");
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
            "p" => {
                if parts.len() != 2 {
                    println!("Usage: p <position>");
                    continue;
                }
                let position: f32 = parts[1].parse()?;
                for &id in &test_ids {
                    let _ = controller.set_position(id, position);
                }
                println!("Set target position to {} for all motors", position);
            }
            "v" => {
                if parts.len() != 2 {
                    println!("Usage: v <velocity>");
                    continue;
                }
                let velocity: f32 = parts[1].parse()?;
                for &id in &test_ids {
                    let _ = controller.set_velocity(id, velocity);
                }
                println!("Set target velocity to {} for all motors", velocity);
            }
            "t" => {
                if parts.len() != 2 {
                    println!("Usage: t <torque>");
                    continue;
                }
                let torque: f32 = parts[1].parse()?;
                for &id in &test_ids {
                    let _ = controller.set_torque(id, torque);
                }
                println!("Set target torque to {} for all motors", torque);
            }
            "kp" => {
                if parts.len() != 2 {
                    println!("Usage: kp <kp>");
                    continue;
                }
                let kp: f32 = parts[1].parse()?;
                for &id in &test_ids {
                    let _ = controller.set_kp(id, kp);
                }
                println!("Set KP to {} for all motors", kp);
            }
            "kd" => {
                if parts.len() != 2 {
                    println!("Usage: kd <kd>");
                    continue;
                }
                let kd: f32 = parts[1].parse()?;
                for &id in &test_ids {
                    let _ = controller.set_kd(id, kd);
                }
                println!("Set KD to {} for all motors", kd);
            }
            "sinusoid" | "s" => {
                let duration = Duration::from_secs(if parts.len() == 2 {
                    parts[1].parse::<u64>()?
                } else {
                    1
                });
                let _ = sinusoid(&controller, &test_ids, 1.0, duration);
                println!("Ran motors {:?} sinusoid test", test_ids);
            }
            "zero" | "z" => {
                for &id in &test_ids {
                    let _ = controller.add_motor_to_zero(id);
                }
                println!("Added motors {:?} to zero list", test_ids);
            }
            "get_feedback" | "g" => {
                let feedback = controller.get_latest_feedback();
                for (id, fb) in feedback {
                    println!("Motor {}: {:?}", id, fb);
                }
            }
            "pause" | "w" => {
                controller.toggle_pause();
                println!("Toggled pause state");
            }
            "reset" | "r" => {
                controller.reset();
                println!("Reset motors");
            }
            "quit" | "q" => {
                controller.stop();
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

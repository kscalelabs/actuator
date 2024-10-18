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
    id: u8,
    amplitude: f32,
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    controller.set_kd(id, 1.0)?;
    controller.set_kp(id, 10.0)?;
    controller.set_velocity(id, 0.0)?;
    controller.set_torque(id, 0.0)?;

    let start = Instant::now();
    let mut last_second = start;
    controller.reset_command_counters();

    while start.elapsed() < duration {
        let t = start.elapsed().as_secs_f32();
        let pos = amplitude * (2.0 * PI * t).sin();
        controller.set_position(id, pos)?;
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

    controller.set_position(id, 0.0)?;

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
    let controller = MotorsSupervisor::new(
        &port_name,
        &motor_infos,
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
                let _ = controller.set_position(test_id, position);
                println!("Set target position to {}", position);
            }
            "v" => {
                if parts.len() != 2 {
                    println!("Usage: v <velocity>");
                    continue;
                }
                let velocity: f32 = parts[1].parse()?;
                let _ = controller.set_velocity(test_id, velocity);
                println!("Set target velocity to {}", velocity);
            }
            "t" => {
                if parts.len() != 2 {
                    println!("Usage: t <torque>");
                    continue;
                }
                let torque: f32 = parts[1].parse()?;
                let _ = controller.set_torque(test_id, torque);
                println!("Set target torque to {}", torque);
            }
            "kp" => {
                if parts.len() != 2 {
                    println!("Usage: kp <kp>");
                    continue;
                }
                let kp: f32 = parts[1].parse()?;
                let _ = controller.set_kp(test_id, kp);
                println!("Set KP for motor {} to {}", test_id, kp);
            }
            "kd" => {
                if parts.len() != 2 {
                    println!("Usage: kd <kd>");
                    continue;
                }
                let kd: f32 = parts[1].parse()?;
                let _ = controller.set_kd(test_id, kd);
                println!("Set KD for motor {} to {}", test_id, kd);
            }
            "sinusoid" | "s" => {
                let duration = Duration::from_secs(if parts.len() == 2 {
                    parts[1].parse::<u64>()?
                } else {
                    1
                });
                let _ = sinusoid(&controller, test_id, 1.0, duration);
                println!("Ran motor {} sinusoid test", test_id);
            }
            "zero" | "z" => {
                let _ = controller.add_motor_to_zero(test_id);
                println!("Added motor {} to zero list", test_id);
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

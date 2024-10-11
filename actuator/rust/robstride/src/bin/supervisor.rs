use robstride::{motor_type_from_str, MotorType, MotorsSupervisor};
use std::collections::HashMap;
use std::io::{self, Write};

fn main() -> Result<(), Box<dyn std::error::Error>> {
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
    let controller = MotorsSupervisor::new(&port_name, &motor_infos)?;

    println!("Motor Controller Test CLI");
    println!("Available commands:");
    println!("  p <position>");
    println!("  v <velocity>");
    println!("  t <torque>");
    println!("  kp <kp>");
    println!("  kd <kd>");
    println!("  zero / z");
    println!("  get_feedback / g");
    println!("  pause / w");
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
                controller.set_position(test_id, position);
                println!("Set target position to {}", position);
            }
            "v" => {
                if parts.len() != 2 {
                    println!("Usage: v <velocity>");
                    continue;
                }
                let velocity: f32 = parts[1].parse()?;
                controller.set_velocity(test_id, velocity);
            }
            "t" => {
                if parts.len() != 2 {
                    println!("Usage: t <torque>");
                    continue;
                }
                let torque: f32 = parts[1].parse()?;
                controller.set_torque(test_id, torque);
            }
            "kp" => {
                if parts.len() != 2 {
                    println!("Usage: kp <kp>");
                    continue;
                }
                let kp: f32 = parts[1].parse()?;
                controller.set_kp(test_id, kp);
                println!("Set KP for motor {} to {}", test_id, kp);
            }
            "kd" => {
                if parts.len() != 2 {
                    println!("Usage: kd <kd>");
                    continue;
                }
                let kd: f32 = parts[1].parse()?;
                controller.set_kd(test_id, kd);
                println!("Set KD for motor {} to {}", test_id, kd);
            }
            "zero" | "z" => {
                controller.add_motor_to_zero(test_id);
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

use robstride::{Motors, RunMode, ROBSTRIDE_CONFIGS};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting program");

    let motors = Motors::new("/dev/ttyUSB0", &[(&ROBSTRIDE_CONFIGS["01"], 2)])?;

    motors.send_set_mode(RunMode::PositionMode)?;
    thread::sleep(Duration::from_millis(50));

    motors.send_reset()?;
    motors.send_start()?;
    thread::sleep(Duration::from_millis(50));

    motors.send_set_speed_limit(5.0)?;
    thread::sleep(Duration::from_millis(50));

    for i in 0..3 {
        motors.send_set_location(std::f32::consts::PI * i as f32 / 2.0)?;
        thread::sleep(Duration::from_secs(1));
    }

    motors.send_set_speed_limit(10.0)?;
    motors.send_set_location(0.0)?;
    thread::sleep(Duration::from_secs(2));

    motors.send_reset()?;

    println!("Program finished");
    Ok(())
}

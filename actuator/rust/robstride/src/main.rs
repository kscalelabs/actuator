use robstride::{Motor, RunMode, ROBSTRIDE01_CONFIG};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("Starting program");

    let mut motor = Motor::new("/dev/ttyCH341USB0", ROBSTRIDE01_CONFIG, 1)?;

    motor.send_set_mode(RunMode::PositionMode)?;
    thread::sleep(Duration::from_millis(50));

    motor.send_reset()?;
    motor.send_start()?;
    thread::sleep(Duration::from_millis(50));

    motor.send_set_speed_limit(5.0)?;
    thread::sleep(Duration::from_millis(50));

    for i in 0..3 {
        motor.send_set_location(std::f32::consts::PI * i as f32 / 2.0)?;
        thread::sleep(Duration::from_secs(1));
    }

    motor.send_set_speed_limit(10.0)?;
    motor.send_set_location(0.0)?;
    thread::sleep(Duration::from_secs(2));

    motor.send_reset()?;

    println!("Program finished");
    Ok(())
}

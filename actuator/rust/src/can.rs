use serialport::SerialPort;
use std::io::{self, Write};
use std::thread::sleep;
use std::time::Duration;

pub fn ping(port_path: Option<&str>) -> io::Result<()> {
    println!("Starting ping function");
    let mut port = match port_path {
        Some(path) => {
            println!("Opening specified port: {}", path);
            open_port(path)?
        }
        None => {
            println!("Finding and opening port");
            find_and_open_port()?
        }
    };

    println!("Initializing CAN channel");
    initialize_can_channel(&mut port)?;

    println!("Sending CAN message");
    let can_message = "t1231001\r";
    port.write_all(can_message.as_bytes())?;
    port.flush()?;

    println!("Reading response");
    read_response(&mut port)?;

    println!("Closing CAN channel");
    port.write_all(b"C\r")?;
    port.flush()?;

    println!("Ping completed successfully");
    Ok(())
}

fn open_port(path: &str) -> io::Result<Box<dyn SerialPort>> {
    println!("Attempting to open port: {}", path);
    serialport::new(path, 115200)
        .timeout(Duration::from_secs(5))
        .data_bits(serialport::DataBits::Eight)
        .flow_control(serialport::FlowControl::None)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .open()
        .map_err(|e| {
            println!("Failed to open port: {}", e);
            io::Error::new(io::ErrorKind::Other, e)
        })
}

fn find_and_open_port() -> io::Result<Box<dyn SerialPort>> {
    let available_ports = serialport::available_ports()?;
    for port_info in available_ports {
        // Skip Bluetooth ports
        if port_info.port_name.contains("Bluetooth") {
            continue;
        }

        // Look for ports that might be our CAN adapter
        // Adjust these conditions based on your specific CAN adapter
        if port_info.port_name.contains("tty.usbmodem")
            || port_info.port_name.contains("ttyACM")
            || port_info.port_name.contains("ttyUSB")
            || port_info.port_name.contains("tty.usbserial")
        // || port_info.port_name.contains("Bluetooth")  // For testing.
        {
            if let Ok(port) = serialport::new(&port_info.port_name, 115200).open() {
                println!("Successfully opened port: {}", port_info.port_name);
                return Ok(port);
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::NotFound,
        "No suitable CAN adapter port found",
    ))
}

fn initialize_can_channel(port: &mut Box<dyn SerialPort>) -> io::Result<()> {
    println!("Initializing CAN channel");
    port.set_timeout(Duration::from_secs(5))?; // Keep 5 second timeout

    // Add a small delay after opening the port
    sleep(Duration::from_millis(100));

    // Try to read any existing data in the buffer
    let mut buffer = [0; 1024];
    match port.read(&mut buffer) {
        Ok(n) => println!("Read {} bytes from buffer: {:?}", n, &buffer[..n]),
        Err(ref e) if e.kind() == io::ErrorKind::TimedOut => println!("No data in buffer"),
        Err(e) => return Err(e),
    }

    // Send reset command
    println!("Sending reset command");
    port.write_all(b"\rC\r")?;
    port.flush()?;
    sleep(Duration::from_millis(100));

    // Open CAN channel
    println!("Sending open channel command");
    port.write_all(b"O\r")?;
    port.flush()?;

    let mut response = [0; 1];
    match port.read_exact(&mut response) {
        Ok(_) => {
            if response[0] == b'\r' {
                println!("CAN channel opened successfully");
                Ok(())
            } else {
                println!(
                    "Unexpected response when opening CAN channel: {:?}",
                    response
                );
                Err(io::Error::new(
                    io::ErrorKind::Other,
                    "Unexpected response when opening CAN channel",
                ))
            }
        }
        Err(e) if e.kind() == io::ErrorKind::TimedOut => {
            println!("Timed out while waiting for CAN channel to open");
            Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "Timed out while waiting for CAN channel to open",
            ))
        }
        Err(e) => {
            println!("Error while opening CAN channel: {}", e);
            Err(e)
        }
    }
}

fn read_response(port: &mut Box<dyn SerialPort>) -> io::Result<()> {
    println!("Reading response");
    port.set_timeout(Duration::from_secs(5))?; // Increase timeout to 5 seconds
    let mut buffer: Vec<u8> = vec![0; 1024];
    match port.read(&mut buffer) {
        Ok(n) => {
            println!("Response received: {:?}", &buffer[..n]);
            Ok(())
        }
        Err(e) if e.kind() == io::ErrorKind::TimedOut => {
            println!("No response received within timeout period");
            Ok(()) // or return Err if you want to treat this as an error
        }
        Err(e) => {
            println!("Error while reading response: {}", e);
            Err(e)
        }
    }
}

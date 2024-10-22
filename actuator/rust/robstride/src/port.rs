use nix::sys::termios;
use serialport::{SerialPort, TTYPort};
use std::os::unix::io::FromRawFd;
use std::time::Duration;

#[cfg(target_os = "linux")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B921600;

// WARNING: NOT A VALID BAUDRATE
// This is just a configuration to build without errors on MacOS
#[cfg(target_os = "macos")]
pub const BAUD_RATE: nix::sys::termios::BaudRate = nix::sys::termios::BaudRate::B115200;

const TIMEOUT: Duration = Duration::from_millis(10);

pub fn init_serial_port(device: &str) -> Result<TTYPort, std::io::Error> {
    // Open the serial port using low-level system calls
    let fd = match nix::fcntl::open(
        device,
        nix::fcntl::OFlag::O_RDWR | nix::fcntl::OFlag::O_NOCTTY | nix::fcntl::OFlag::O_NDELAY,
        nix::sys::stat::Mode::empty(),
    ) {
        Ok(fd) => fd,
        Err(e) => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to open serial port {}: {}", device, e),
            ));
        }
    };

    // Get the current flags of the file descriptor
    let mut flags = match nix::fcntl::fcntl(fd, nix::fcntl::FcntlArg::F_GETFL) {
        Ok(flags) => flags,
        Err(e) => {
            let _ = nix::unistd::close(fd);
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to get file descriptor flags: {}", e),
            ));
        }
    };

    // Clear the O_NONBLOCK flag to enable blocking I/O
    flags &= !nix::fcntl::OFlag::O_NONBLOCK.bits();

    // Set the updated flags
    if let Err(e) = nix::fcntl::fcntl(
        fd,
        nix::fcntl::FcntlArg::F_SETFL(nix::fcntl::OFlag::from_bits_truncate(flags)),
    ) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set file descriptor flags: {}", e),
        ));
    }

    // Get the current termios (terminal I/O) settings
    let mut termios = match termios::tcgetattr(fd) {
        Ok(termios) => termios,
        Err(e) => {
            // If getting termios settings fails, close the file descriptor and return an error
            let _ = nix::unistd::close(fd);
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to get termios settings: {}", e),
            ));
        }
    };

    // Set the input baud rate
    if let Err(e) = termios::cfsetispeed(&mut termios, BAUD_RATE) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set input baud rate: {}", e),
        ));
    }
    // Set the output baud rate
    if let Err(e) = termios::cfsetospeed(&mut termios, BAUD_RATE) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set output baud rate: {}", e),
        ));
    }

    // Configure port settings:
    // Clear parity bit, stop bits, and character size flags
    termios.control_flags &= !(termios::ControlFlags::PARENB
        | termios::ControlFlags::CSTOPB
        | termios::ControlFlags::CSIZE);

    // Set 8-bit characters, enable receiver, ignore modem control lines
    termios.control_flags |=
        termios::ControlFlags::CS8 | termios::ControlFlags::CREAD | termios::ControlFlags::CLOCAL;

    // Set raw input/output mode:
    // Disable canonical mode, echo, erasure, and signal chars
    termios.local_flags &= !(termios::LocalFlags::ICANON
        | termios::LocalFlags::ECHO
        | termios::LocalFlags::ECHOE
        | termios::LocalFlags::ISIG);

    // Clear all input flags
    termios.input_flags = termios::InputFlags::empty();

    // Disable implementation-defined output processing
    termios.output_flags &= !termios::OutputFlags::OPOST;

    // Set VMIN (minimum number of characters for noncanonical read) to 0
    termios.control_chars[termios::SpecialCharacterIndices::VMIN as usize] = 0;

    // Set VTIME (timeout in deciseconds for noncanonical read) to 1 second
    termios.control_chars[termios::SpecialCharacterIndices::VTIME as usize] = 10;

    // Apply the new termios settings
    if let Err(e) = termios::tcsetattr(fd, termios::SetArg::TCSANOW, &termios) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set termios attributes: {}", e),
        ));
    }

    // Verify that the settings were applied correctly
    let verify_termios = match termios::tcgetattr(fd) {
        Ok(termios) => termios,
        Err(e) => {
            let _ = nix::unistd::close(fd);
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to verify termios settings: {}", e),
            ));
        }
    };
    if verify_termios.control_flags != termios.control_flags
        || verify_termios.input_flags != termios.input_flags
        || verify_termios.output_flags != termios.output_flags
        || verify_termios.local_flags != termios.local_flags
    {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Serial port settings do not match the requested configuration",
        ));
    }

    // Flush both input and output buffers of the serial port
    if let Err(e) = termios::tcflush(fd, termios::FlushArg::TCIOFLUSH) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to flush serial port: {}", e),
        ));
    }

    // Create a TTYPort from the raw file descriptor
    let mut port = unsafe { TTYPort::from_raw_fd(fd) };

    // Set the timeout for read/write operations
    port.set_timeout(TIMEOUT)?;

    // Return the configured TTYPort
    Ok(port)
}

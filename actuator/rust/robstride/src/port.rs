use nix::sys::termios;
use serialport::{SerialPort, TTYPort};
use std::os::unix::io::FromRawFd;
use std::time::Duration;

const BAUD_RATE: termios::BaudRate = termios::BaudRate::B921600;
const TIMEOUT: Duration = Duration::from_millis(10);

pub fn init_serial_port(device: &str) -> Result<TTYPort, std::io::Error> {
    // Open the serial port
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
            ))
        }
    };

    // Clear the O_NONBLOCK flag
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
    flags &= !nix::fcntl::OFlag::O_NONBLOCK.bits();
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

    // Get current termios settings
    let mut termios = match termios::tcgetattr(fd) {
        Ok(termios) => termios,
        Err(e) => {
            let _ = nix::unistd::close(fd);
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("Failed to get termios settings: {}", e),
            ));
        }
    };

    // Set input and output baud rates
    if let Err(e) = termios::cfsetispeed(&mut termios, BAUD_RATE) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set input baud rate: {}", e),
        ));
    }
    if let Err(e) = termios::cfsetospeed(&mut termios, BAUD_RATE) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set output baud rate: {}", e),
        ));
    }

    // Configure port settings
    termios.control_flags &= !(termios::ControlFlags::PARENB
        | termios::ControlFlags::CSTOPB
        | termios::ControlFlags::CSIZE);
    termios.control_flags |=
        termios::ControlFlags::CS8 | termios::ControlFlags::CREAD | termios::ControlFlags::CLOCAL;

    // Set raw input/output mode
    termios.local_flags &= !(termios::LocalFlags::ICANON
        | termios::LocalFlags::ECHO
        | termios::LocalFlags::ECHOE
        | termios::LocalFlags::ISIG);
    termios.input_flags = termios::InputFlags::empty();
    termios.output_flags &= !termios::OutputFlags::OPOST;

    // Set VMIN and VTIME
    termios.control_chars[termios::SpecialCharacterIndices::VMIN as usize] = 0;
    termios.control_chars[termios::SpecialCharacterIndices::VTIME as usize] = 10;

    // Apply the settings
    if let Err(e) = termios::tcsetattr(fd, termios::SetArg::TCSANOW, &termios) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to set termios attributes: {}", e),
        ));
    }

    // Verify the settings
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
    if verify_termios != termios {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            "Serial port settings do not match the requested configuration",
        ));
    }

    // Flush the serial port
    if let Err(e) = termios::tcflush(fd, termios::FlushArg::TCIOFLUSH) {
        let _ = nix::unistd::close(fd);
        return Err(std::io::Error::new(
            std::io::ErrorKind::Other,
            format!("Failed to flush serial port: {}", e),
        ));
    }

    let mut port = unsafe { TTYPort::from_raw_fd(fd) };
    port.set_timeout(TIMEOUT)?;

    Ok(port)
}

use eyre::Error;
use serialport::SerialPort;
#[cfg(target_os = "linux")]
use socketcan::{CanSocket, EmbeddedFrame, ExtendedId};
use std::io::{Read, Write};
use std::sync::Mutex;

/// Result type for send operations
type SendResult = Result<(), Error>;
/// Result type for receive operations
type RecvResult = Result<(u32, Vec<u8>), Error>;

#[derive(Clone)]
pub enum TransportType {
    CH341(CH341Transport),
    #[cfg(target_os = "linux")]
    SocketCAN(SocketCanTransport),
    Stub(StubTransport),
}

impl Transport for TransportType {
    fn kind(&self) -> &'static str {
        match self {
            TransportType::CH341(t) => t.kind(),
            #[cfg(target_os = "linux")]
            TransportType::SocketCAN(t) => t.kind(),
            TransportType::Stub(t) => t.kind(),
        }
    }

    fn port(&self) -> String {
        match self {
            TransportType::CH341(t) => t.port(),
            #[cfg(target_os = "linux")]
            TransportType::SocketCAN(t) => t.port(),
            TransportType::Stub(t) => t.port(),
        }
    }

    fn send(&mut self, id: u32, data: &[u8]) -> SendResult {
        match self {
            TransportType::CH341(t) => t.send(id, data),
            #[cfg(target_os = "linux")]
            TransportType::SocketCAN(t) => t.send(id, data),
            TransportType::Stub(t) => t.send(id, data),
        }
    }

    fn recv(&mut self) -> RecvResult {
        match self {
            TransportType::CH341(t) => t.recv(),
            #[cfg(target_os = "linux")]
            TransportType::SocketCAN(t) => t.recv(),
            TransportType::Stub(t) => t.recv(),
        }
    }

    fn clear_buffer(&mut self) -> Result<(), Error> {
        match self {
            TransportType::CH341(t) => t.clear_buffer(),
            #[cfg(target_os = "linux")]
            TransportType::SocketCAN(t) => t.clear_buffer(),
            TransportType::Stub(t) => t.clear_buffer(),
        }
    }
}

pub trait Transport {
    fn kind(&self) -> &'static str;
    fn port(&self) -> String;
    fn send(&mut self, id: u32, data: &[u8]) -> SendResult;
    fn recv(&mut self) -> RecvResult;
    fn clear_buffer(&mut self) -> Result<(), Error>;
}

pub struct CH341Transport {
    ser: Mutex<Box<dyn SerialPort>>,
    port_name: String,
}

#[cfg(target_os = "linux")]
pub struct SocketCanTransport {
    socket: Mutex<CanSocket>,
    interface_name: String,
}

pub struct StubTransport {
    last_command: Option<(u32, Vec<u8>)>,
}

impl CH341Transport {
    pub fn new(port_name: String) -> Result<Self, Error> {
        let ser = serialport::new(&port_name, 921600)
            .timeout(std::time::Duration::from_millis(100))
            .open()?;
        Ok(Self {
            ser: Mutex::new(ser),
            port_name,
        })
    }
}

#[cfg(target_os = "linux")]
impl SocketCanTransport {
    pub fn new(interface_name: String) -> Result<Self, Error> {
        let socket = CanSocket::open(&interface_name)?;
        Ok(Self {
            socket: Mutex::new(socket),
            interface_name,
        })
    }
}

impl StubTransport {
    pub fn new() -> Self {
        Self { last_command: None }
    }
}

impl Transport for CH341Transport {
    fn send(&mut self, id: u32, data: &[u8]) -> SendResult {
        let mut pkt = Vec::new();
        pkt.extend_from_slice(b"AT");
        let addr = (id << 3) | 0x4;
        pkt.extend_from_slice(&addr.to_be_bytes());
        pkt.push(data.len() as u8);
        pkt.extend_from_slice(data);
        pkt.extend_from_slice(b"\r\n");

        {
            let mut ser = self.ser.lock().unwrap();
            ser.write_all(&pkt)?;
        }
        std::thread::sleep(std::time::Duration::from_nanos(20));
        Ok(())
    }

    fn recv(&mut self) -> RecvResult {
        let mut buf = vec![0; 1024];
        let mut pos = 0;

        loop {
            let n = {
                let mut ser = self.ser.lock().unwrap();
                ser.read(&mut buf[pos..])?
            };

            if n == 0 {
                return Err(eyre::eyre!("EOF"));
            }
            pos += n;

            for i in 0..pos.saturating_sub(7) {
                if buf[i] == b'A' && buf[i + 1] == b'T' {
                    if let Ok((id, data, _msg_len)) = parse_message(&buf[i..pos]) {
                        return Ok((id, data));
                    }
                }
            }

            if pos >= buf.len() - 8 {
                return Err(eyre::eyre!("Buffer full without finding valid message"));
            }
        }
    }

    fn clear_buffer(&mut self) -> Result<(), Error> {
        let mut ser = self.ser.lock().unwrap();
        let mut buf = vec![0; 1024];
        loop {
            match ser.read(&mut buf) {
                Ok(0) => break,    // No more data
                Ok(_) => continue, // Discard data
                Err(_) => break,   // Error or timeout
            }
        }
        Ok(())
    }

    fn kind(&self) -> &'static str {
        "CH341"
    }

    fn port(&self) -> String {
        self.port_name.clone()
    }
}

// Helper function to parse a single message
fn parse_message(buf: &[u8]) -> Result<(u32, Vec<u8>, usize), Error> {
    // Ensure we have at least the minimum length for a valid packet
    // AT + 4 bytes ID + 1 byte length + \r\n = 8 bytes minimum
    if buf.len() < 8 {
        return Err(eyre::eyre!("Buffer too short"));
    }

    // Verify AT prefix
    if buf[0] != b'A' || buf[1] != b'T' {
        return Err(eyre::eyre!("Invalid AT prefix"));
    }

    // Get data length
    let data_len = buf[6] as usize;

    // Calculate total message length
    let total_len = 7 + data_len + 2; // AT + ID + len + data + \r\n

    // Ensure we have enough bytes for the complete packet
    if buf.len() < total_len {
        return Err(eyre::eyre!("Incomplete message"));
    }

    // Check \r\n termination
    if buf[total_len - 2] != b'\r' || buf[total_len - 1] != b'\n' {
        return Err(eyre::eyre!("Invalid message termination"));
    }

    // Extract CAN ID (4 bytes, big endian)
    let mut id_bytes = [0u8; 4];
    id_bytes.copy_from_slice(&buf[2..6]);
    let raw_id = u32::from_be_bytes(id_bytes);

    // Undo the encoding: shift right by 3 and clear the highest 3 bits
    let id = (raw_id >> 3) & 0x1FFF_FFFF; // Mask to 29 bits (extended CAN ID)

    // Extract data
    let data = buf[7..7 + data_len].to_vec();

    Ok((id, data, total_len))
}

#[cfg(target_os = "linux")]
impl Transport for SocketCanTransport {
    fn send(&mut self, id: u32, data: &[u8]) -> SendResult {
        let extended_id =
            ExtendedId::new(id).ok_or_else(|| eyre::eyre!("Invalid CAN ID: {}", id))?;
        let msg = socketcan::CanFrame::new(extended_id, data)
            .ok_or_else(|| eyre::eyre!("Failed to create CAN frame"))?;

        {
            let mut socket = self.socket.lock().unwrap();
            socket.write_frame(&msg)?;
        }
        Ok(())
    }

    fn recv(&mut self) -> RecvResult {
        let frame = {
            let mut socket = self.socket.lock().unwrap();
            socket.read_frame()?
        };

        let id = match frame.id() {
            socketcan::Id::Standard(id) => id.as_raw() as u32,
            socketcan::Id::Extended(id) => id.as_raw(),
        };
        Ok((id, frame.data().to_vec()))
    }

    fn clear_buffer(&mut self) -> Result<(), Error> {
        let mut socket = self.socket.lock().unwrap();
        loop {
            match socket.read_frame() {
                Ok(_) => continue, // Discard frame
                Err(_) => break,   // Error or timeout
            }
        }
        Ok(())
    }

    fn kind(&self) -> &'static str {
        "SocketCAN"
    }

    fn port(&self) -> String {
        self.interface_name.clone()
    }
}

impl Transport for StubTransport {
    fn port(&self) -> String {
        "stub".to_string()
    }

    fn kind(&self) -> &'static str {
        "Stub"
    }

    fn send(&mut self, id: u32, data: &[u8]) -> SendResult {
        // Store the command for later retrieval
        self.last_command = Some((id, data.to_vec()));
        Ok(())
    }

    fn recv(&mut self) -> RecvResult {
        // Return the last command if available, otherwise return a default response
        let (id, data) = self.last_command.take().unwrap_or_else(|| {
            (
                0x2000100,
                vec![0x7f, 0xfe, 0x80, 0x73, 0x7f, 0xff, 0x01, 0x18],
            )
        });

        std::thread::sleep(std::time::Duration::from_millis(100));
        Ok((id, data))
    }

    fn clear_buffer(&mut self) -> Result<(), Error> {
        // Clear the stored command
        self.last_command = None;
        Ok(())
    }
}

impl Clone for CH341Transport {
    fn clone(&self) -> Self {
        // Note: This is a simplified clone that creates a new connection
        // In practice, you might want to handle this differently
        Self {
            ser: Mutex::new(
                serialport::new(&self.port_name, 921600)
                    .timeout(std::time::Duration::from_millis(100))
                    .open()
                    .unwrap_or_else(|_| panic!("Failed to clone CH341Transport")),
            ),
            port_name: self.port_name.clone(),
        }
    }
}

#[cfg(target_os = "linux")]
impl Clone for SocketCanTransport {
    fn clone(&self) -> Self {
        // Note: This is a simplified clone that creates a new connection
        // In practice, you might want to handle this differently
        Self {
            socket: Mutex::new(
                CanSocket::open(&self.interface_name)
                    .unwrap_or_else(|_| panic!("Failed to clone SocketCanTransport")),
            ),
            interface_name: self.interface_name.clone(),
        }
    }
}

impl Clone for StubTransport {
    fn clone(&self) -> Self {
        Self {
            last_command: self.last_command.clone(),
        }
    }
}

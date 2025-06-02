use eyre::Error;
use socketcan::async_std::CanSocket;
use socketcan::{EmbeddedFrame, ExtendedId};
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::Mutex as TokioMutex;
use tokio_serial::{SerialPortBuilderExt, SerialStream};

/// Result type for send operations
type SendResult = Result<(), Error>;
/// Result type for receive operations
type RecvResult = Result<(u32, Vec<u8>), Error>;
/// Future type for send operations
type SendFuture<'a> = std::pin::Pin<Box<dyn std::future::Future<Output = SendResult> + Send + 'a>>;
/// Future type for receive operations
type RecvFuture<'a> = std::pin::Pin<Box<dyn std::future::Future<Output = RecvResult> + Send + 'a>>;

#[derive(Clone)]
pub enum TransportType {
    CH341(CH341Transport),
    SocketCAN(SocketCanTransport),
    Stub(StubTransport),
}

impl Transport for TransportType {
    fn kind(&self) -> &'static str {
        match self {
            TransportType::CH341(t) => t.kind(),
            TransportType::SocketCAN(t) => t.kind(),
            TransportType::Stub(t) => t.kind(),
        }
    }

    fn port(&self) -> String {
        match self {
            TransportType::CH341(t) => t.port(),
            TransportType::SocketCAN(t) => t.port(),
            TransportType::Stub(t) => t.port(),
        }
    }

    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        match self {
            TransportType::CH341(t) => t.send(id, data),
            TransportType::SocketCAN(t) => t.send(id, data),
            TransportType::Stub(t) => t.send(id, data),
        }
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        match self {
            TransportType::CH341(t) => t.recv(),
            TransportType::SocketCAN(t) => t.recv(),
            TransportType::Stub(t) => t.recv(),
        }
    }
}

pub trait Transport {
    fn kind(&self) -> &'static str;
    fn port(&self) -> String;
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a>;
    fn recv(&mut self) -> RecvFuture<'_>;
}

pub struct CH341Transport {
    ser: Arc<TokioMutex<SerialStream>>,
    port_name: String,
}

pub struct SocketCanTransport {
    socket: Arc<TokioMutex<CanSocket>>,
    interface_name: String,
}

pub struct StubTransport {
    port_name: String,
}

impl CH341Transport {
    pub async fn new(port_name: String) -> Result<Self, Error> {
        let ser = tokio_serial::new(&port_name, 921600).open_native_async()?;
        Ok(Self {
            ser: Arc::new(TokioMutex::new(ser)),
            port_name,
        })
    }
}

impl SocketCanTransport {
    pub async fn new(interface_name: String) -> Result<Self, Error> {
        let socket = CanSocket::open(&interface_name)?;
        Ok(Self {
            socket: Arc::new(TokioMutex::new(socket)),
            interface_name,
        })
    }
}

impl StubTransport {
    pub fn new(port_name: String) -> Self {
        Self { port_name }
    }
}

impl Transport for CH341Transport {
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        let ser = self.ser.clone();
        Box::pin(async move {
            let mut pkt = Vec::new();
            pkt.extend_from_slice(b"AT");
            let addr = (id << 3) | 0x4;
            pkt.extend_from_slice(&addr.to_be_bytes());
            pkt.push(data.len() as u8);
            pkt.extend_from_slice(data);
            pkt.extend_from_slice(b"\r\n");

            {
                let mut ser = ser.lock().await;
                ser.write_all(&pkt).await?;
            }
            tokio::time::sleep(tokio::time::Duration::from_nanos(20)).await;
            Ok(())
        })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let ser = self.ser.clone();
        Box::pin(async move {
            let mut buf = vec![0; 1024];
            let mut pos = 0;

            loop {
                let n = {
                    let mut ser = ser.lock().await;
                    ser.read(&mut buf[pos..]).await?
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
        })
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

impl Transport for SocketCanTransport {
    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        let socket = self.socket.clone();
        Box::pin(async move {
            let extended_id =
                ExtendedId::new(id).ok_or_else(|| eyre::eyre!("Invalid CAN ID: {}", id))?;
            let msg = socketcan::CanFrame::new(extended_id, data)
                .ok_or_else(|| eyre::eyre!("Failed to create CAN frame"))?;

            {
                let socket = socket.lock().await;
                socket.write_frame(&msg).await?;
            }
            Ok(())
        })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let socket = self.socket.clone();
        Box::pin(async move {
            let frame = {
                let socket = socket.lock().await;
                socket.read_frame().await?
            };

            let id = match frame.id() {
                socketcan::Id::Standard(id) => id.as_raw() as u32,
                socketcan::Id::Extended(id) => id.as_raw(),
            };
            Ok((id, frame.data().to_vec()))
        })
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
        self.port_name.clone()
    }

    fn kind(&self) -> &'static str {
        "Stub"
    }

    fn send<'a>(&'a mut self, id: u32, data: &'a [u8]) -> SendFuture<'a> {
        tracing::trace!("StubTransport::send: id={:04x}, data={:02x?}", id, data);
        Box::pin(async move { Ok(()) })
    }

    fn recv(&mut self) -> RecvFuture<'_> {
        let id = 0x2000100;
        let data = vec![0x7f, 0xfe, 0x80, 0x73, 0x7f, 0xff, 0x01, 0x18];
        // tracing::trace!("StubTransport::recv: id={:04x}, data={:02x?}", id, data);
        Box::pin(async move {
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
            Ok((id, data))
        })
    }
}

impl Clone for CH341Transport {
    fn clone(&self) -> Self {
        Self {
            ser: self.ser.clone(),
            port_name: self.port_name.clone(),
        }
    }
}

impl Clone for SocketCanTransport {
    fn clone(&self) -> Self {
        Self {
            socket: self.socket.clone(),
            interface_name: self.interface_name.clone(),
        }
    }
}

impl Clone for StubTransport {
    fn clone(&self) -> Self {
        Self {
            port_name: self.port_name.clone(),
        }
    }
}

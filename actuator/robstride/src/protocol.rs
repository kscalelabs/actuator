use crate::transport::{Transport, TransportType};
use eyre::Error;
use std::sync::Arc;
use tracing::trace;

pub struct Protocol {
    transport: TransportType,
    callback: Arc<dyn Fn(u32, Vec<u8>) + Send + Sync + 'static>,
}

impl Protocol {
    pub fn new(
        transport: TransportType,
        callback: Arc<dyn Fn(u32, Vec<u8>) + Send + Sync + 'static>,
    ) -> Self {
        Self {
            transport,
            callback,
        }
    }

    pub fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Error> {
        trace!(
            "send {}:{} {:x}: {:02x?}",
            self.transport.kind(),
            self.transport.port(),
            id,
            data
        );
        self.transport.send(id, data)
    }

    pub fn recv(&mut self) -> Result<(u32, Vec<u8>), Error> {
        let (id, data) = self.transport.recv()?;
        trace!(
            "recv {}:{} {:x}: {:02x?}",
            self.transport.kind(),
            self.transport.port(),
            id,
            data
        );

        (self.callback)(id, data.clone());

        Ok((id, data))
    }

    pub fn clear_buffer(&mut self) -> Result<(), Error> {
        self.transport.clear_buffer()
    }
}

impl Clone for Protocol {
    fn clone(&self) -> Self {
        Self {
            transport: self.transport.clone(),
            callback: self.callback.clone(),
        }
    }
}

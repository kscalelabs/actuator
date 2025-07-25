use crate::transport::{Transport, TransportType};
use eyre::Error;
use std::sync::Arc;
use tracing::trace;


impl std::fmt::Debug for Protocol {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Protocol {{ transport: {}, callback: <function> }}",
            self.transport.kind()
        )
    }
}

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

    pub async fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Error> {
        // trace!(
        //     "send {}:{} {:x}: {:02x?}",
        //     self.transport.kind(),
        //     self.transport.port(),
        //     id,
        //     data
        // );
        self.transport.send(id, data).await
    }

    pub async fn recv(&mut self) -> Result<(u32, Vec<u8>), Error> {
        let (id, data) = self.transport.recv().await?;
        // trace!(
        //     "recv {}:{} {:x}: {:02x?}",
        //     self.transport.kind(),
        //     self.transport.port(),
        //     id,
        //     data
        // );

        (self.callback)(id, data.clone());

        Ok((id, data))
    }

    pub async fn process_incoming(&mut self) -> Result<(), Error> {
        loop {
            let (_id, _data) = self.recv().await?;
        }
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

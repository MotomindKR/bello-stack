use std::io;
use std::time::Duration;

use anyhow::{Context, Result};
use bello_sim_can::SimSocket;

use super::super::BusAccess;
use super::blocking::BlockingSocket;
use super::{BusSpec, CanFrame, FrameWriter, ReceivedFrame, WriteFailure};
use crate::config::BusTransportConfig;
use crate::domain::BusName;

impl FrameWriter for SimSocket {
    fn write_frame(&self, frame: CanFrame) -> Result<(), WriteFailure> {
        self.send(frame.into()).map_err(|error| WriteFailure {
            queue_full: matches!(
                error.kind(),
                io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut
            ),
            detail: format!("sim CAN write failed: {error}"),
        })
    }

    fn fence_writes(&self) -> Result<(), String> {
        Ok(())
    }
}

impl BlockingSocket for SimSocket {
    type Writer = Self;

    fn open_pair(spec: &BusSpec) -> Result<(Self, Self)> {
        let BusTransportConfig::Sim(bus) = &spec.transport else {
            unreachable!("sim CAN supervisor requires a sim transport")
        };
        let socket = bus
            .open_daemon()
            .with_context(|| format!("opening {} ({})", spec.name, spec.transport.label()))?;
        socket.set_read_timeout(Some(Duration::from_millis(100)))?;
        Ok((socket.try_clone()?, socket))
    }

    fn receive(&self, bus: &BusName, _: &BusAccess) -> io::Result<Option<ReceivedFrame>> {
        let (frame, received) = self.recv_timestamped()?;
        Ok(Some(ReceivedFrame {
            bus: bus.clone(),
            frame: frame.into(),
            received,
        }))
    }
}

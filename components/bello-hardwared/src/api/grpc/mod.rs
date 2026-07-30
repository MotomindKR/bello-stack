use std::pin::Pin;

use futures_core::Stream;
use tonic::Status;

mod command_stream;
mod conversion;
mod inventory;
mod service;
mod spec;
mod state_frame;
mod telemetry_stream;

pub use service::HardwareGrpc;
pub use spec::{GrpcListenSpec, GrpcSpec};

type ResponseStream<T> = Pin<Box<dyn Stream<Item = Result<T, Status>> + Send + 'static>>;

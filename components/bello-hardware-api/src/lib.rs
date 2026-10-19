//! Shared protobuf and gRPC contract for Bello hardware services.

// tonic-generated async_trait methods carry a redundant must_use attribute.
#![allow(clippy::double_must_use)]

tonic::include_proto!("bello.hardware.v1");

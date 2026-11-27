// tonic-generated async_trait methods carry a redundant must_use attribute.
#![allow(clippy::double_must_use)]

tonic::include_proto!("bello.hardware.v1");

pub mod camera;
pub mod deployment;

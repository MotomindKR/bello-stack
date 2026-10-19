//! Neutral streaming pose contract shared by producers and display clients.

// tonic-generated async_trait methods carry a redundant must_use attribute.
#![allow(clippy::double_must_use)]

tonic::include_proto!("bello.pose.v1");

pub const FILE_DESCRIPTOR_SET: &[u8] =
    tonic::include_file_descriptor_set!("bello_pose_v1_descriptor");

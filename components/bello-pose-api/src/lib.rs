//! Neutral streaming pose contract shared by producers and display clients.

tonic::include_proto!("bello.pose.v1");

pub const FILE_DESCRIPTOR_SET: &[u8] =
    tonic::include_file_descriptor_set!("bello_pose_v1_descriptor");

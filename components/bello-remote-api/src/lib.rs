//! Neutral normalized remote-command contract shared by producers and policy clients.

tonic::include_proto!("bello.remote.v1");

pub const FILE_DESCRIPTOR_SET: &[u8] =
    tonic::include_file_descriptor_set!("bello_remote_v1_descriptor");

use std::sync::Arc;

use tokio::sync::mpsc;
use tokio_stream::StreamExt;
use tokio_stream::wrappers::ReceiverStream;

use super::{ResponseStream, conversion};
use crate::app::HardwareDaemon;
use crate::proto::{StreamCommandsRequest, StreamCommandsResponse};

pub fn open(
    daemon: Arc<HardwareDaemon>,
    mut input: tonic::Streaming<StreamCommandsRequest>,
) -> ResponseStream<StreamCommandsResponse> {
    let (tx, rx) = mpsc::channel(1024);
    tokio::spawn(async move {
        while let Some(next) = input.next().await {
            let request = match next {
                Ok(request) => request,
                Err(error) => {
                    let _ = tx.send(Err(error)).await;
                    break;
                }
            };
            let batch = match conversion::command_batch(request) {
                Ok(batch) => batch,
                Err(error) => {
                    let _ = tx.send(Err(error)).await;
                    break;
                }
            };
            let group = batch.group.clone();
            let sequence = batch.sequence;
            let result = daemon
                .submit_command(batch)
                .await
                .map_err(|error| error.to_string());
            let state = daemon
                .groups
                .get(&group)
                .map(|group| group.snapshot.read().state.as_str().to_owned())
                .unwrap_or_default();
            let response = StreamCommandsResponse {
                group: group.to_string(),
                sequence,
                accepted: result.is_ok(),
                message: result.err().unwrap_or_else(|| "accepted".into()),
                state,
            };
            if tx.send(Ok(response)).await.is_err() {
                break;
            }
        }
    });
    Box::pin(ReceiverStream::new(rx))
}

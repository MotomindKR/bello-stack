use std::sync::Arc;

use tokio::sync::watch;
use tokio_stream::StreamExt;

use super::{ResponseStream, conversion};
use crate::app::HardwareDaemon;
use bello_hardware_api::{CommandResult, StreamCommandsRequest, StreamCommandsResponse};

pub fn open(
    daemon: Arc<HardwareDaemon>,
    mut input: tonic::Streaming<StreamCommandsRequest>,
    mut shutdown: watch::Receiver<bool>,
) -> ResponseStream<StreamCommandsResponse> {
    Box::pin(async_stream::stream! {
        loop {
            let next = tokio::select! {
                biased;
                changed = shutdown.changed() => {
                    if changed.is_err() || *shutdown.borrow() {
                        break;
                    }
                    continue;
                }
                next = input.next() => next,
            };
            let Some(next) = next else { break };
            let request = match next {
                Ok(request) => request,
                Err(error) => {
                    yield Err(error);
                    break;
                }
            };
            let batch = match conversion::command_batch(request) {
                Ok(batch) => batch,
                Err(error) => {
                    yield Err(error);
                    break;
                }
            };
            let group = batch.group.clone();
            let sequence = batch.sequence;
            let (result, detail) = match daemon.submit_command(batch).await {
                Ok(()) => (
                    CommandResult::Accepted as i32,
                    "accepted".to_owned(),
                ),
                Err(error) => (conversion::command_result(&error), error.to_string()),
            };
            let state = daemon
                .groups
                .get(&group)
                .map(|group| conversion::group_state(group.snapshot.read().state))
                .unwrap_or_default();
            yield Ok(StreamCommandsResponse {
                group: group.to_string(),
                sequence,
                result,
                detail,
                state,
            });
        }
    })
}

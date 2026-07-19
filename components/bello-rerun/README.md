# Bello Rerun observer

`bello-rerun` is an optional standalone visualization and recording process. It
connects to `bello-hardwared` through the public read-only gRPC API and never
acquires a lease or submits commands. The hardware daemon therefore has no
Rerun dependency, feature, or configuration.

## Run

Validate the observer configuration without connecting or creating output:

```sh
cargo run -p bello-rerun -- \
  --check --config components/bello-rerun/config/bello.yaml
```

Start the observer:

```sh
cargo run -p bello-rerun -- \
  --config components/bello-rerun/config/bello.yaml
```

Omitting `--config` selects this component's `config/bello.yaml` example.

## Configuration

The maintained example saves an `.rrd` recording:

```yaml
hardware_endpoint: http://127.0.0.1:50051

reconnect:
  initial_delay_ms: 100
  maximum_delay_ms: 5000

output:
  mode: save
  path: bello_hardware.rrd
```

To stream to an existing Rerun listener, use:

```yaml
output:
  mode: connect
  address: 127.0.0.1:9876
```

The address is a bare `host:port` with a port in `1..=65535`, without a URL
scheme or path. The repository's Nix shell does not install or start the
external listener.

Configuration is strict and rejects unknown fields. Reconnect delays use
bounded exponential backoff; the defaults are 100 ms initially and 5000 ms at
the maximum. See the complete
[configuration reference](../../docs/configuration.md#rerun-observer-configuration).
HTTPS hardware endpoints use native certificate roots and HTTP/2. Keepalive is
sent every 10 seconds with a three-second timeout, even while idle.

## Runtime behavior

The observer requests every telemetry class through the read-only
`WatchTelemetry` stream. It can start before the daemon and reconnects
indefinitely after unavailable or interrupted connections. Connection and
daemon-restart markers are recorded alongside hardware telemetry.

Source `monotonic_ns` values drive the Rerun timeline. If hardwared restarts and
its monotonic clock moves backwards, the observer rebases the new segment after
the previous event while retaining the restart marker. Subscriber-drop health
events remain visible. Malformed optional event fields are logged and skipped
without ending the stream.

The async gRPC task uses nonblocking `try_send` into a fixed 4096-event queue.
A dedicated OS thread owns telemetry mapping, timeline state, and every Rerun
SDK call. If that queue fills, the observer drops events and warns once for the
burst; after the writer resumes it adds a health marker with the dropped count.
Rerun backpressure therefore cannot block gRPC reception or signal polling.
Each telemetry item carries its connection generation, so the first surviving
event still restores the reconnect boundary used for timeline rebasing.

SIGINT and SIGTERM are handled independently of hardwared. On either signal,
the writer attempts to drain queued work and flush output; main waits at most
two seconds before detaching an unfinished writer. Failure to initialize the
selected output is fatal. This component is diagnostic, not an authoritative
safety or forensic log.

## Source layout

```text
src/main.rs      CLI and Tokio runtime entry
src/config.rs    strict YAML model and validation
src/observer.rs  gRPC connection, subscription, and reconnect
src/writer.rs    bounded queue, timeline ownership, and Rerun SDK thread
src/mapping.rs   protobuf telemetry to Rerun entities
src/timeline.rs  monotonic source-time preservation and restart rebasing
src/backoff.rs   bounded reconnect delay state
```

See [Operations](../../docs/operations.md#rerun-observation) for deployment and
the [API guide](../../docs/api.md#inventory-and-telemetry) for stream semantics.

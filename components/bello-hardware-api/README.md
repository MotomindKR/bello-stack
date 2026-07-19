# Bello hardware API

`bello-hardware-api` owns the versioned protobuf contract shared by
`bello-hardwared`, read-only observers, and integration tests. It generates the
tonic client and server types at build time and contains no runtime service or
hardware behavior.

Rust applications should use the robot-oriented
[`bello-hardware-client`](../bello-hardware-client). This crate remains the raw
wire contract and is available from that client only through the explicit
`bello_hardware_client::raw::proto` escape hatch.

The authoritative source is
[`proto/bello/hardware/v1/hardware.proto`](proto/bello/hardware/v1/hardware.proto).
Cargo uses the `protoc` executable from `PATH`; the repository Nix environment
provides it. Format and lint schema changes from the repository root:

```sh
buf format --diff --exit-code
buf lint
```

See the [API guide](../../docs/api.md) for workflows and telemetry semantics.

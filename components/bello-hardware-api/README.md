# Bello hardware API

[hardware.proto](proto/bello/hardware/v1/hardware.proto) defines the gRPC interface to
`bello-hardwared`. Rust applications can use [bello-hardware-client](../bello-hardware-client/)
or import the generated types from `bello_hardware_client::raw::proto`.

Cargo uses the `protoc` executable from `PATH`; the repository Nix environment
provides it. Format and lint schema changes from the repository root:

```sh
buf format --diff --exit-code
buf lint
```

See the [API guide](../../docs/api.md) for workflows and telemetry semantics.

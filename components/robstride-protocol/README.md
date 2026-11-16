# Robstride protocol

`robstride-protocol` is the shared, dependency-light Robstride extended-CAN
library used by hardwared and the emulator.

It provides:

- semantic request and feedback types;
- extended CAN identifier and eight-byte payload encoding;
- parsing for the protocol operations used by this workspace;
- control command conversion using model-specific ranges;
- supported motor-model limits and direction handling.

```text
src/
  model.rs   generated-catalog interface and command conversion parameters
  frame.rs   raw extended-CAN frame representation and address helpers
  codec.rs   semantic request, feedback, and command codecs
  lib.rs     intentionally small public surface
```

The crate does not open SocketCAN, own devices, implement safety policy, or
schedule control. Those responsibilities belong to
[`bello-hardwared`](../bello-hardwared).

[`descriptions/robstride/models.yaml`](../../descriptions/robstride/models.yaml)
is the sole handwritten model catalog. The build script validates it and
generates the protocol lookup table and shared Type-1 position constants.
`ModelLimits` also exposes the catalog's optional typed hardware metadata
(no-load speed, rated voltage, and peak phase current) instead of discarding it
at build time.

`ROBSTRIDE_10P` uses the RS10P CAN 2.0 private-protocol ranges. This model name
does not select CAN-FD or the separate standard-ID MIT protocol. See the
[Milly 7-DOF notes](../../descriptions/milly-7dof/README.md#commissioning-values)
for the manufacturer source and installation-limit distinction.

Run its focused tests with:

```sh
cargo test -p robstride-protocol
```

See the [hardware guide](../../docs/hardware.md) for its position in the runtime
stack.

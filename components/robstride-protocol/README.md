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
  model.rs   model capabilities and command conversion parameters
  frame.rs   raw extended-CAN frame representation and address helpers
  codec.rs   semantic request, feedback, and command codecs
  lib.rs     intentionally small public surface
```

The crate does not open SocketCAN, own devices, implement safety policy, or
schedule control. Those responsibilities belong to
[`bello-hardwared`](../bello-hardwared).

Run its focused tests with:

```sh
cargo test -p robstride-protocol
```

See the [hardware guide](../../docs/hardware.md) for its position in the runtime
stack.

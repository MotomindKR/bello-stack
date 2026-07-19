# Bello hardware end-to-end tests

This component contains an ignored Cargo integration test that runs the real
`bello-hardwared` and `robstride-emulator` implementations in child processes.
It communicates only through the public gRPC API and a host SocketCAN `vcan`
interface.

The default scenario:

1. generates matching runtime configurations and starts both processes;
2. acquires a group lease through gRPC;
3. enables two emulated Robstride actuators;
4. streams complete impedance-command snapshots and starts control;
5. suspends the emulator to begin a deterministic feedback-loss phase;
6. resumes it, verifies process liveness, and confirms the group remains fault-latched;
7. explicitly recovers and re-enables the group;
8. stops command delivery and observes the command watchdog fault;
9. recovers, disables the group, and synchronously releases its lease;
10. starts under a short second lease and observes lease-expiry fault handling.

Create `vcan0` once per boot if needed:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Run from the Nix/direnv development environment:

```sh
cargo test -p bello-hardware-e2e --test vcan -- --ignored --nocapture
```

`cargo e2e` is an alias for the same command. The ignored test is compiled but
not executed by ordinary `cargo test`. When explicitly run, it fails rather than
silently skipping if its host prerequisite is absent. It selects an unused
loopback gRPC port for each run.

Set `BELLO_E2E_CAN_INTERFACE` to use an interface other than `vcan0`. The
selected name is validated and written into both generated runtime
configurations.

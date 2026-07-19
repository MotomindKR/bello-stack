# Bello hardware end-to-end tests

This component runs behavioral integration scenarios against the actual
`bello-hardwared` and `robstride-emulator` processes. It communicates only through
the public gRPC API and a real SocketCAN `vcan` interface.

The default scenario:

1. starts both binaries;
2. acquires a group lease through gRPC;
3. enables two emulated Robstride actuators;
4. streams complete impedance-command snapshots and starts control;
5. drops one actuator's feedback and observes the group fault telemetry;
6. resumes feedback and verifies the group remains fault-latched;
7. explicitly recovers and re-enables the group;
8. disables the group and releases its lease.

Create `vcan0` once per boot if needed:

```sh
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Run from the Nix/direnv development environment:

```sh
cargo e2e
```

The Cargo alias builds the daemon and emulator binaries before starting the
scenario. The runner fails rather than silently skipping when its vcan
prerequisite is missing. It uses port `50052`, separately from the normal
hardwared example.

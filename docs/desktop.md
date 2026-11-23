# Desktop builds

`bello-hardwared` and its protocol and client libraries support
Linux, macOS, and Windows. GS-USB is available on all three; SocketCAN is compiled
only on Linux. A SocketCAN deployment is rejected by `--check` and startup on
other operating systems before any hardware is opened.

## Native executables

The Desktop GitHub Actions workflow builds and tests the portable components on
Linux x86-64, macOS Apple Silicon, and Windows x86-64. Each successful job uploads
a `bello-desktop-<platform>` artifact containing the release daemon, robot
descriptions and meshes, deployment examples, and the license.
Extract the entire archive and keep its directory layout. Rust and `protoc` are
needed only to build from source.

Linux artifacts are dynamically linked and built on Ubuntu 24.04. Binaries
built inside Nix can reference the Nix store. Use the native CI artifacts when
distributing to hosts without Nix. macOS artifacts are unsigned.

For Milly, edit `components/bello-hardwared/config/milly-gsusb.yaml` and replace
`REPLACE_WITH_ADAPTER_SERIAL` with the adapter's USB serial number. Select its
channel (normally `0`). The canonical hardware description supplies the CAN
bitrate, motor IDs, models, directions, and limits. The same deployment works on
all three operating systems.

From the extracted archive directory, run:

```sh
./bello-hardwared --check --config components/bello-hardwared/config/milly-gsusb.yaml
./bello-hardwared --config components/bello-hardwared/config/milly-gsusb.yaml
```

In Windows PowerShell, use `./bello-hardwared.exe`. The Milly profile listens on
TCP at `127.0.0.1:50051`. Stop the daemon with Ctrl-C for orderly shutdown.

Other robot profiles use the same transport mapping. Serial device names are
host specific: use `COM3` on Windows, `/dev/cu.*` on macOS, or stable
`/dev/serial/by-id/*` paths on Linux. TCP gRPC works on all three operating systems;
Unix socket listeners are available on Linux and macOS.

## USB setup

- **Linux:** grant the process read/write access to the adapter's USB device
  node using a udev rule. GS-USB temporarily detaches `gs_usb` while it owns the
  USB interface. Use one owner per adapter.
- **Windows:** associate the CAN adapter's interface with **WinUSB**, either
  through firmware descriptors or with Zadig. Select the CAN adapter interface.
- **macOS:** nusb uses IOKit, normally without permission changes. An interface
  held by a kernel driver cannot be claimed. Vendor-class firmware may need
  explicit USB configuration selection; verify discovery and opening on the
  intended adapter.

See the [nusb platform notes](https://docs.rs/nusb/latest/nusb/#platform-support).
Native CI validates compilation, software tests, and the packaged executables;
physical USB timing, driver setup, and reconnect behavior still need validation
on each target host. The Milly GS-USB path has been exercised on Linux with live
feedback from seven motors.

## Build from source

Install Rust 1.95 or newer, a native C/C++ toolchain, and `protoc` on `PATH`.
On Windows use the MSVC Rust toolchain and Visual Studio C++ Build Tools. On
macOS install the Xcode command line tools. On Linux install `pkg-config` and
the libudev development headers, or use the Nix development shell.

```sh
cargo build --locked --release
cargo test --locked -p bello-hardwared -p bello-hardware-client
python scripts/package-desktop.py --platform linux-x86_64
```

The package script reads the release daemon from `CARGO_TARGET_DIR/release`
or `target/release`. Pass `--binary-dir` when using a different output directory.
Use a lowercase platform label such as `windows-x86_64`. The workspace builds
`bello-hardwared` by default. Select other programs with `-p`.
Run the full workspace and vCAN checks on Linux for the MuJoCo host integration
and vCAN process tests.

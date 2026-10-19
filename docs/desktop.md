# Desktop builds

`bello-hardwared`, `bello-visualizer`, and their protocol/client libraries support
Linux, macOS, and Windows. GS-USB is available on all three; SocketCAN is compiled
only on Linux. A SocketCAN deployment is rejected by `--check` and startup on
other operating systems before any hardware is opened.

## Native executables

The Desktop GitHub Actions workflow builds and tests the portable components on
Linux x86-64, macOS Apple Silicon, and Windows x86-64. Each successful job uploads
a `bello-desktop-<platform>` artifact containing a ZIP with two release
executables, the robot descriptions and meshes, deployment examples, and license.
Extract the entire archive and keep its directory layout. Rust and `protoc` are
needed only to build from source.

The visualizer uses the host's graphics drivers. Linux also needs its desktop
graphics libraries (Wayland/X11, xkbcommon, EGL/Vulkan, and udev). These are
native desktop builds with an Ubuntu 24.04 baseline, not statically linked Linux
distributions. Binaries built inside Nix can also reference the Nix store;
use the native CI artifacts when distributing to hosts without Nix. macOS artifacts
are unsigned; signed/notarized application bundles are not produced by this
workflow.

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

In another terminal:

```sh
./bello-visualizer --config components/bello-visualizer/config/milly.example.yaml
```

On Windows PowerShell, use `./bello-hardwared.exe` and
`./bello-visualizer.exe`. Both examples use TCP on `127.0.0.1:50051`.
The visualizer starts in monitoring mode; **Commanding** acquires a lease and
enables the selected motors. Stop the daemon with Ctrl-C for orderly shutdown.

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
macOS install the Xcode command line tools. The Linux Nix shell supplies the
Linux toolchain and graphics libraries; it is not required on macOS or Windows.

```sh
cargo build --locked --release
cargo test --locked -p bello-hardwared -p bello-hardware-client -p bello-visualizer
python scripts/package-desktop.py --platform linux-x86_64
```

The package script reads release executables from `CARGO_TARGET_DIR/release`
or `target/release`. Pass `--binary-dir` when using a different output directory.
Use a lowercase platform label such as `windows-x86_64`. The workspace defaults
to the two desktop applications; select other programs explicitly with `-p`.
Run the full workspace
and vCAN checks on Linux: the MuJoCo host integration and vCAN process tests
remain Linux-specific.

{
  description = "Development environment for the Bello hardware stack";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          visualizerRuntimeLibraries = with pkgs; [
            wayland
            libxkbcommon
            libx11
            libxcursor
            libxi
            libxrandr
            udev
            libglvnd
            vulkan-loader
          ];
          pythonWheelRuntimeLibraries = [
            pkgs.stdenv.cc.cc.lib
          ];
          gstreamerPackages = with pkgs.gst_all_1; [
            gstreamer
            gst-plugins-base
            gst-plugins-good
            gst-plugins-bad
            gst-plugins-ugly
            gst-libav
          ];
          runtimeLibraries =
            visualizerRuntimeLibraries ++ pythonWheelRuntimeLibraries ++
            gstreamerPackages;
        in {
          default = pkgs.mkShell {
            packages = with pkgs; [
              cargo
              rustc
              rustfmt
              clippy
              protobuf
              pkg-config
              openssl
              grpcurl
              can-utils
              iproute2
              buf
              glib
              gobject-introspection
              (python3.withPackages (python: with python; [
                grpcio
                grpcio-tools
                hatchling
                mypy
                numpy
                pygobject3
                pyyaml
                pytest
                types-pyyaml
              ]))
              ruff
            ] ++ visualizerRuntimeLibraries ++ gstreamerPackages;
            RUST_BACKTRACE = "1";
            # Keep Cargo artifacts outside this non-Git path flake. Otherwise Nix
            # copies target/ into every source snapshot before entering the shell.
            shellHook = ''
              export CARGO_TARGET_DIR="''${XDG_CACHE_HOME:-''${HOME}/.cache}/bello-stack/cargo-target"
              export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath runtimeLibraries}:''${LD_LIBRARY_PATH:-}"
            '';
          };
        });
    };
}

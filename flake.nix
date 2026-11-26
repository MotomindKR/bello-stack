{
  description = "Development environment for the Bello hardware stack";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" "aarch64-darwin" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          inherit (pkgs.stdenv.hostPlatform) isLinux;
          mediaPlugins = with pkgs.gst_all_1; [ gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad ];
          runtimeLibraries = with pkgs; [
            stdenv.cc.cc.lib
            libglvnd
            mesa
            mujoco
          ];
        in {
          default = pkgs.mkShell ({
            packages = with pkgs; [
              cargo
              rustc
              rustfmt
              clippy
              protobuf
              pkg-config
              openssl
              grpcurl
              buf
              (python3.withPackages (python: with python; [
                grpcio
                grpcio-tools
                hatchling
                mypy
                pytest
              ]))
              ruff
              uv
            ] ++ mediaPlugins ++ pkgs.lib.optionals isLinux (with pkgs; [
              can-utils
              ffmpeg
              iproute2
              mujoco
            ]);
            GST_PLUGIN_SYSTEM_PATH_1_0 = pkgs.lib.makeSearchPath "lib/gstreamer-1.0" mediaPlugins;
            RUST_BACKTRACE = "1";
            # Path flakes copy in-tree Cargo artifacts into the Nix store.
            shellHook = ''
              export CARGO_TARGET_DIR="''${XDG_CACHE_HOME:-''${HOME}/.cache}/bello-stack/cargo-target"
            '' + pkgs.lib.optionalString isLinux ''
              export __EGL_VENDOR_LIBRARY_DIRS="''${__EGL_VENDOR_LIBRARY_DIRS:-${pkgs.mesa}/share/glvnd/egl_vendor.d}"
              export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath runtimeLibraries}:''${LD_LIBRARY_PATH:-}"
            '';
          } // pkgs.lib.optionalAttrs isLinux {
            MUJOCO_DYNAMIC_LINK_DIR = assert pkgs.mujoco.version == "3.12.0";
              "${pkgs.lib.getLib pkgs.mujoco}/lib";
          });
        });
    };
}

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
            libglvnd
            vulkan-loader
          ];
          pythonWheelRuntimeLibraries = [
            pkgs.stdenv.cc.cc.lib
          ];
          runtimeLibraries =
            visualizerRuntimeLibraries ++ pythonWheelRuntimeLibraries;
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
              (python3.withPackages (python: with python; [
                grpcio
                grpcio-tools
                hatchling
                mypy
                pytest
              ]))
              ruff
            ] ++ visualizerRuntimeLibraries;
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

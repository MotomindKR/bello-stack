{
  description = "Development environment for the Bello hardware stack";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/06a21ded0b150acacc0d132ff61e6693d4c771de";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          runtimeLibraries = with pkgs; [
            stdenv.cc.cc.lib
            libglvnd
            mujoco
          ];
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
              mujoco
              (python3.withPackages (python: with python; [
                grpcio
                grpcio-tools
                hatchling
                mypy
                pytest
              ]))
              ruff
            ];
            RUST_BACKTRACE = "1";
            MUJOCO_DYNAMIC_LINK_DIR = assert pkgs.mujoco.version == "3.12.0";
              "${pkgs.lib.getLib pkgs.mujoco}/lib";
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

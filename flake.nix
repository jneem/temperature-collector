{
  description = "Devshell for esp32c3 dev";

  inputs = {
    nixpkgs.url = "nixpkgs";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, rust-overlay, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
        };
        rust-toolchain = pkgs.rust-bin.nightly.latest.default.override {
          extensions = [ "llvm-tools-preview" "rust-src" ];
          targets = [ "riscv32imc-unknown-none-elf" ];
        };

        probeRs = pkgs.rustPlatform.buildRustPackage rec {
          pname = "probe-rs";
          version = "c9f959e";
          cargoHash = "sha256-ZOXfIVvLlvB+GNv4E6dS7hNIaEzBD4ObgVKSvcB4xfs=";
          buildFeatures = [ "cli" ];
          doCheck = false;

          src = pkgs.fetchFromGitHub {
            owner = "probe-rs";
            repo = "probe-rs";
            rev = version;
            hash = "sha256-GlVABpgj9G7UP+4Q31n/zUxvg/128HOpJH0uZO6mhfQ=";
          };

          nativeBuildInputs = [ pkgs.pkg-config ];
          buildInputs = [ pkgs.libusb1 pkgs.openssl ];
        };
      in
      {
        devShell = pkgs.mkShell {
          nativeBuildInputs = with pkgs; [
            rust-toolchain
            rust-analyzer
            cargo-espflash
            cargo-outdated
            cargo-expand
            taplo
            probeRs
            pkg-config
            udev
          ];
        };

        packages =
          let mkPackage = pkgs: pkgs.rustPlatform.buildRustPackage {
            pname = "temperature-collector";
            version = "20230911";
            src = ./collector;
            cargoLock = {
              lockFile = ./collector/Cargo.lock;
            };
          };
        in
        {
          default = mkPackage pkgs;
          cross-aarch64 = mkPackage pkgs.pkgsCross.aarch64-multiplatform;
        };
      }
    );
}

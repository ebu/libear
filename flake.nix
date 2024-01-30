{
  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.nixpkgs.url = "nixpkgs/nixos-23.11";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = import nixpkgs {
            system = system;
            config.allowUnsupportedSystem = true; # for eigen wasm
          };
          pkgs_wasm = pkgs.pkgsCross.wasi32;

          devtools = [
            pkgs.clang-tools
            pkgs.cmake-format
            pkgs.nixpkgs-fmt
          ];
        in
        rec {
          packages.xsimd = pkgs.xsimd.overrideAttrs rec {
            version = "12.1.1";
            src = pkgs.fetchFromGitHub {
              owner = "xtensor-stack";
              repo = "xsimd";
              rev = "12.1.1";
              sha256 = "sha256-ofUFieeRtpnzNv3Ad5oYwKWb2XcqQHoj601TIhydJyI=";
            };
          };
          packages.libear = pkgs.callPackage ./nix/libear.nix { src = ./.; xsimd = packages.xsimd; };
          packages.default = packages.libear;

          packages.libear_clang = packages.libear.override { stdenv = pkgs.clangStdenv; };

          devShells.libear = packages.libear.overrideAttrs (attrs: {
            nativeBuildInputs = attrs.nativeBuildInputs ++ devtools;
          });
          devShells.default = devShells.libear;

          # wasm versions of packages

          packages.xsimd_wasm = pkgs_wasm.xsimd.overrideAttrs {
            # this should be an overlay
            version = pkgs.xsimd.version;
            src = pkgs.xsimd.src;
          };
          packages.libear_wasm = (pkgs_wasm.callPackage ./nix/libear.nix {
            src = ./.;
            boost = pkgs.boost; # doesn't build for wasm, but we only use headers, so use system version
            xsimd = packages.xsimd_wasm;
          }).overrideAttrs (super: {
            cmakeBuildType = "MinSizeRel";
          });

          devShells.libear_wasm = packages.libear_wasm.overrideAttrs (attrs: {
            nativeBuildInputs = attrs.nativeBuildInputs ++ devtools ++ [
              pkgs.wabt
              pkgs.wasmtime
              pkgs.nodejs
              pkgs.nodePackages.prettier
            ];
          });
        }
      );
}


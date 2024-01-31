{
  inputs.flake-utils.url = "github:numtide/flake-utils";
  inputs.nixpkgs.url = "nixpkgs/nixos-23.11";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = nixpkgs.legacyPackages.${system};

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

          packages.libear_ci = packages.libear.overrideAttrs { cmakeBuildType = "CI"; };
          packages.libear_clang_ci = packages.libear_clang.overrideAttrs { cmakeBuildType = "CI"; };

          devShells.libear = packages.libear.overrideAttrs (attrs: {
            nativeBuildInputs = attrs.nativeBuildInputs ++ devtools;
          });
          devShells.default = devShells.libear;
        }
      );
}


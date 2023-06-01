{ lib, buildPackages, stdenv, cmake, src, ninja, boost, eigen, xsimd }:
let
  isCross = stdenv.buildPlatform != stdenv.hostPlatform;
in
(stdenv.mkDerivation {
  name = "libear";
  inherit src;
  nativeBuildInputs = [
    cmake
    ninja
  ];
  buildInputs = [ boost eigen xsimd ];
  cmakeFlags = [
    "-DEAR_USE_INTERNAL_EIGEN=OFF"
    "-DEAR_USE_INTERNAL_XSIMD=OFF"
    "-DEAR_UNIT_TESTS=ON"
  ]
  ++ lib.optionals isCross [
    "-DCMAKE_CROSSCOMPILING_EMULATOR=${stdenv.hostPlatform.emulator buildPackages}"
  ];

  doCheck = true;
})

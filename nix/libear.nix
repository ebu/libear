{ lib, buildPackages, stdenv, cmake, src, ninja, boost, eigen, xsimd, binaryen, nodejs }:
let
  isCross = stdenv.buildPlatform != stdenv.hostPlatform;
  isWasm = stdenv.hostPlatform.isWasm;
in
(stdenv.mkDerivation {
  name = "libear";
  inherit src;
  nativeBuildInputs = [
    cmake
    ninja
  ] ++ lib.optionals isWasm [
    binaryen # gets used automatically by clang-ld if in path
    nodejs
  ];

  buildInputs = [ boost eigen xsimd ];
  cmakeFlags = [
    "-DEAR_USE_INTERNAL_EIGEN=OFF"
    "-DEAR_USE_INTERNAL_XSIMD=OFF"
    "-DEAR_UNIT_TESTS=ON"
  ]
  ++ lib.optionals isCross [
    "-DCMAKE_CROSSCOMPILING_EMULATOR=${stdenv.hostPlatform.emulator buildPackages}"
  ] ++ lib.optionals isWasm [
    "-DEAR_NO_EXCEPTIONS=ON"
  ];


  doCheck = true;

  preConfigure = lib.optionalString isWasm ''
    # for wasmtime cache
    HOME=$(pwd)
    # forced off in make-derivation.nix when build platform can't execute host
    # platform, but we have an emulator
    doCheck=1
  '';

  env.NIX_CFLAGS_COMPILE = lib.optionalString isWasm "-DEIGEN_HAS_CXX11_ATOMIC=0 -DCATCH_CONFIG_NO_POSIX_SIGNALS";
})

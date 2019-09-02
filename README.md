# libear — Recommendation ITU-R BS.2127 core library

*libear* is a C++11 library to render ADM content according to [Recommendation
ITU-R BS.2127](https://www.itu.int/rec/R-REC-BS.2127). It is not a complete
application, but provides components to calculate gains and apply them to
audio, for embedding into applications which need to render ADM content.

[Read the documentation](https://libear.readthedocs.io/en/latest/) to get
started.

## Installation

### Dependencies

  - compiler with C++11 support
  - Boost header libraries (version 1.57 or later)
      - Boost.Optional
      - Boost.Variant
  - CMake build system (version 3.5 or later)

### Installation

To manually install the library you have to recursively clone the git
repository and then use the CMake build system to build and install it.

``` shell
git clone --recursive https://github.com/ebu/libear.git
cd libear
mkdir build && cd build
cmake ..
make
make install
```

### Use from CMake Projects

As the library uses CMake as a build system it is really easy to set up
and use if your project does too. Assuming you have installed the
library, the following code shows a complete CMake example to compile a
program which uses the libear.

``` cmake
cmake_minimum_required(VERSION 3.5)
project(libear_example VERSION 1.0.0 LANGUAGES CXX)

find_package(ear REQUIRED)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE ear)
```

### Use as a Subproject

If you prefer not to install the library on your system you can also use
the library as a CMake subproject. Just add the folder containing the
repository to your project (for example, by using a git submodule) and
you can use the ear target:

``` cmake
cmake_minimum_required(VERSION 3.5)
project(libear_example VERSION 1.0.0 LANGUAGES CXX)

add_subdirectory(submodules/libear)

add_executable(example example.cpp)
target_link_libraries(example PRIVATE ear)
```

Note that if `libear` is used as a CMake subproject the default values of the
options

  - `EAR_UNIT_TESTS`
  - `EAR_EXAMPLES`
  - `EAR_PACKAGE_AND_INSTALL`

are automatically set to `FALSE`.

## License

```
Copyright 2019 The libear Authors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

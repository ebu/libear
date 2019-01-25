Installation
============

Dependencies
------------

-  compiler with C++11 support
-  Boost header libraries (version 1.57 or later)

   -  Boost.Optional
   -  Boost.Variant

-  CMake build system (version 3.5 or later)

-  yaml-cpp_ (version 0.5.2 or later)

.. _yaml-cpp: https://github.com/jbeder/yaml-cpp

Installation
------------

To manually install the library you have to recursively clone the git
repository and then use the CMake build system to build and install it.

.. code-block:: shell

   git clone --recursive https://github.com/ebu/libear.git
   cd libear
   mkdir build && cd build
   cmake ..
   make
   make install

Use from CMake Projects
-----------------------

As the library uses CMake as a build system it is really easy to set up
and use if your project does too. Assuming you have installed the
library, the following code shows a complete CMake example to compile a
program which uses the libear.

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.5)
   project(libear_example VERSION 1.0.0 LANGUAGES CXX)

   find_package(Boost 1.57 REQUIRED)
   find_package(ear REQUIRED)

   add_executable(example example.cpp)
   target_link_libraries(example PRIVATE ear)

Use as a Subproject
-------------------

If you prefer not to install the library on your system you can also use the
library as a CMake subproject. Just add the folder containing the repository to
your project (for example, by using a git submodule) and you can use the ear
target:

.. code-block:: cmake

   cmake_minimum_required(VERSION 3.5)
   project(libear_example VERSION 1.0.0 LANGUAGES CXX)

   add_subdirectory(submodules/libear)

   add_executable(example example.cpp)
   target_link_libraries(example PRIVATE ear)

.. note::

   If ``libear`` is used as a CMake subproject the default values of the
   options

   -  ``EAR_UNIT_TESTS``
   -  ``EAR_EXAMPLES``
   -  ``EAR_PACKAGE_AND_INSTALL``

   are automatically set to ``FALSE``.

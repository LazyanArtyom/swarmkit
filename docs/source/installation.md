# Installation

## Requirements

SwarmKit is built as a modern C++23 CMake project.

| Requirement | Version or note |
| --- | --- |
| C++ compiler | GCC 13+, Clang 17+, or Xcode Command Line Tools on macOS ARM64 |
| CMake | 3.28 or newer for the main project |
| Ninja | Recommended generator used by the shipped presets |
| Conan | Conan 2 for dependency resolution |
| Python | Needed for Conan and documentation builds |
| Doxygen | Needed for API reference XML |

Runtime/build dependencies are resolved by Conan:

- gRPC 1.67.1
- Protobuf 5.27.0
- spdlog 1.17.0
- yaml-cpp 0.8.0
- Catch2 3.8.0 when tests are enabled

The MAVLink C headers used by the direct MAVLink backend are vendored under
`third_party/mavlink_c_library_v2`.

Initialize Conan once:

```bash
conan profile detect
```

## Build From Source

On macOS ARM64:

```bash
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset mac-debug
cmake --build --preset mac-debug
```

For a release build:

```bash
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset mac-release
cmake --build --preset mac-release
```

On Linux x86_64:

```bash
conan install . -of build/conan -s build_type=Debug -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset linux-debug
cmake --build --preset linux-debug
```

For a release build:

```bash
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 \
  -o '&:with_tools=True' -o '&:with_tests=True' --build=missing
cmake --preset linux-release
cmake --build --preset linux-release
```

## Run Tests

```bash
ctest --preset mac-release --output-on-failure
ctest --preset linux-release --output-on-failure
```

Use the preset matching your host platform and build type.

## Install The SDK

After building a release tree, install the SDK component into a prefix:

```bash
cmake --install build/mac-release --component sdk --prefix /tmp/swarmkit-sdk
```

The install exports headers, generated protocol headers, static libraries, and
CMake package files under `lib/cmake/SwarmKit`.

The tools package can be installed separately:

```bash
cmake --install build/mac-release --component tools --prefix /tmp/swarmkit-tools
```

## Use From Another CMake Project

Create a consumer `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.24)
project(SwarmKitConsumer CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SwarmKit REQUIRED)

add_executable(quick_start main.cpp)
target_link_libraries(quick_start PRIVATE swarmkit::client)
```

Configure with the SDK prefix:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/tmp/swarmkit-sdk
cmake --build build
```

Other exported targets include `swarmkit::core`, `swarmkit::proto`, and
`swarmkit::agent`.

## Build The Documentation

Install Doxygen with your system package manager first:

```bash
# macOS
brew install doxygen

# Ubuntu/Debian
sudo apt-get update
sudo apt-get install -y doxygen graphviz
```

Then build the Sphinx site:

```bash
cd docs
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
make html
open build/html/index.html
```

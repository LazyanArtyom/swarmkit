cmake_minimum_required(VERSION 3.28)

# ---- Version (single source of truth) ----
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" SWARMKIT_VERSION)
string(STRIP "${SWARMKIT_VERSION}" SWARMKIT_VERSION)

project(swarmkit VERSION ${SWARMKIT_VERSION} LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(SwarmkitOptions)
include(SwarmkitWarnings)
include(SwarmkitSanitizers)

option(SWARMKIT_BUILD_TOOLS "Build agent+cli executables" ON)
option(SWARMKIT_BUILD_TESTS "Build unit tests" ON)

find_package(fmt CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(ZLIB REQUIRED)

# ---- Canonical generated root (shared by version + proto) ----
set(SWARMKIT_GEN_ROOT "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Generated public header (installed as part of SDK)
file(MAKE_DIRECTORY "${SWARMKIT_GEN_ROOT}/swarmkit/core")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/include/swarmkit/core/version.h.in"
  "${SWARMKIT_GEN_ROOT}/swarmkit/core/version.h"
)

add_subdirectory(proto)
add_subdirectory(src/core)
add_subdirectory(src/client)
add_subdirectory(src/agent)

if (SWARMKIT_BUILD_TOOLS)
  add_subdirectory(apps)
endif()

include(CTest)
if (BUILD_TESTING AND SWARMKIT_BUILD_TESTS)
  find_package(Catch2 CONFIG REQUIRED)
  add_subdirectory(tests)
endif()

# ---- Install / Export (SDK) ----
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Public headers (hand-written)
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/include/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
        COMPONENT sdk
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.in" EXCLUDE
        PATTERN ".DS_Store" EXCLUDE)

# Generated headers (version.h + generated proto headers)
install(DIRECTORY "${SWARMKIT_GEN_ROOT}/swarmkit/"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/swarmkit"
        COMPONENT sdk
        FILES_MATCHING PATTERN "*.h")

# Install SDK libraries
install(TARGETS
    swarmkit_core
    swarmkit_proto
    swarmkit_client
    swarmkit_agent_runtime
    EXPORT SwarmKitTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT sdk
)

# CMake package files
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/SwarmKitConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/SwarmKitConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SwarmKit"
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/SwarmKitConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/SwarmKitConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/SwarmKitConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SwarmKit"
  COMPONENT sdk
)

install(EXPORT SwarmKitTargets
  NAMESPACE swarmkit::
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/SwarmKit"
  COMPONENT sdk
)

# Tools install (separate component)
if (SWARMKIT_BUILD_TOOLS)
  install(TARGETS swarmkit-agent swarmkit-cli
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
          COMPONENT tools)
endif()

# ---- Packaging ----
include("${CMAKE_CURRENT_SOURCE_DIR}/packaging/cpack.cmake")
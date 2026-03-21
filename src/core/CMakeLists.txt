add_library(swarmkit_core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/crc32.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/logger.cpp
)

target_include_directories(swarmkit_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:include>
)

# crc32.h is an internal header used only within the repository.
target_include_directories(swarmkit_core PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)

target_link_libraries(swarmkit_core PUBLIC
    fmt::fmt
    spdlog::spdlog
    ZLIB::ZLIB
)

set_target_properties(swarmkit_core PROPERTIES EXPORT_NAME core)

swarmkit_set_warnings(swarmkit_core)
swarmkit_enable_sanitizers(swarmkit_core)

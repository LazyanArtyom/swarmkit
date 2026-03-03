add_library(swarmkit_core STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../scenario/scenario_yaml.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/crc32.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/logger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/mock_backend.cpp
)

target_include_directories(swarmkit_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/generated>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_core PUBLIC
    fmt::fmt
    spdlog::spdlog
    yaml-cpp::yaml-cpp
    ZLIB::ZLIB
)

set_target_properties(swarmkit_core PROPERTIES EXPORT_NAME core)

swarmkit_set_warnings(swarmkit_core)
swarmkit_enable_sanitizers(swarmkit_core)
# packaging/cpack.cmake

set(CPACK_PACKAGE_NAME "swarmkit")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")

# Make TGZ/ZIP extract into a single top-level directory (instead of include/, lib/ in CWD)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_PACKAGING_INSTALL_PREFIX "/")

# Nice OS tags instead of Darwin
if (WIN32)
  set(CPACK_SYSTEM_NAME "win")
elseif(APPLE)
  set(CPACK_SYSTEM_NAME "mac")
else()
  set(CPACK_SYSTEM_NAME "linux")
endif()

# Portable archives
if (WIN32)
  set(CPACK_GENERATOR "ZIP")
else()
  set(CPACK_GENERATOR "TGZ")
endif()

# Components
set(CPACK_COMPONENTS_ALL sdk tools)
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

set(CPACK_COMPONENT_SDK_DISPLAY_NAME "SwarmKit SDK")
set(CPACK_COMPONENT_TOOLS_DISPLAY_NAME "SwarmKit Tools")

# Put output archives into build/<preset>/packages/
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_BINARY_DIR}/packages")

include(CPack)
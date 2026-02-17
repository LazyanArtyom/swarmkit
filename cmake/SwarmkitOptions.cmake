if(DEFINED SWARMKIT_OPTIONS_INCLUDED)
    return()
endif()
set(SWARMKIT_OPTIONS_INCLUDED TRUE)

option(SWARMKIT_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(SWARMKIT_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(SWARMKIT_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)

# Logging defaults
set(SWARMKIT_DEFAULT_LOG_LEVEL "info" CACHE STRING "Default log level: trace,debug,info,warn,error,critical,off")
set(SWARMKIT_DEFAULT_LOG_DIR "" CACHE STRING "If set, agent logs go to files in this directory")
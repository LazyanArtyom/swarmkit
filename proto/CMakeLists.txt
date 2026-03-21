# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

cmake_minimum_required(VERSION 3.28)

add_library(swarmkit_proto STATIC)

set(proto_root_dir "${CMAKE_CURRENT_LIST_DIR}")
set(proto_file "${proto_root_dir}/swarmkit.proto")

# Canonical generated root for the whole build tree
set(gen_root "${CMAKE_BINARY_DIR}/generated")
set(gen_pkg  "${gen_root}/swarmkit/v1")

file(MAKE_DIRECTORY "${gen_pkg}")

# C++ protoc outputs go directly into --cpp_out directory (no package folders),
# so we explicitly set output dir to gen_pkg to get generated/swarmkit/v1/...
set(proto_hdr "${gen_pkg}/swarmkit.pb.h")
set(proto_src "${gen_pkg}/swarmkit.pb.cc")
set(grpc_hdr  "${gen_pkg}/swarmkit.grpc.pb.h")
set(grpc_src  "${gen_pkg}/swarmkit.grpc.pb.cc")

get_target_property(protoc_path protobuf::protoc IMPORTED_LOCATION)
get_target_property(grpc_cpp_plugin_path gRPC::grpc_cpp_plugin IMPORTED_LOCATION)

add_custom_command(
  OUTPUT "${proto_hdr}" "${proto_src}" "${grpc_hdr}" "${grpc_src}"
  COMMAND "${protoc_path}"
          --proto_path=${proto_root_dir}
          --cpp_out=${gen_pkg}
          --grpc_out=${gen_pkg}
          --plugin=protoc-gen-grpc=${grpc_cpp_plugin_path}
          ${proto_file}
  DEPENDS "${proto_file}"
  COMMENT "Generating protobuf/gRPC sources from ${proto_file}"
  VERBATIM
)

target_sources(swarmkit_proto PRIVATE
  "${proto_src}"
  "${grpc_src}"
)

# Include from gen_root so code can do: #include "swarmkit/v1/swarmkit.pb.h"
target_include_directories(swarmkit_proto PUBLIC
  $<BUILD_INTERFACE:${gen_root}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(swarmkit_proto PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
)

set_target_properties(swarmkit_proto PROPERTIES EXPORT_NAME proto)

swarmkit_set_warnings(swarmkit_proto)
swarmkit_enable_sanitizers(swarmkit_proto)

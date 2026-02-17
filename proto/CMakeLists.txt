# proto/CMakeLists.txt
cmake_minimum_required(VERSION 3.28)

add_library(swarmkit_proto STATIC)

set(proto_root_dir ${CMAKE_CURRENT_LIST_DIR})  # <repo>/proto
set(proto_file ${proto_root_dir}/swarmkit.proto)

set(generated_dir ${CMAKE_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${generated_dir})

set(proto_src ${generated_dir}/swarmkit.pb.cc)
set(proto_hdr ${generated_dir}/swarmkit.pb.h)
set(grpc_src ${generated_dir}/swarmkit.grpc.pb.cc)
set(grpc_hdr ${generated_dir}/swarmkit.grpc.pb.h)

get_target_property(protoc_path protobuf::protoc IMPORTED_LOCATION)
get_target_property(grpc_cpp_plugin_path gRPC::grpc_cpp_plugin IMPORTED_LOCATION)

add_custom_command(
    OUTPUT
        ${proto_src}
        ${proto_hdr}
        ${grpc_src}
        ${grpc_hdr}
    COMMAND
        ${protoc_path}
        --proto_path=${proto_root_dir}
        --cpp_out=${generated_dir}
        --grpc_out=${generated_dir}
        --plugin=protoc-gen-grpc=${grpc_cpp_plugin_path}
        ${proto_file}
    DEPENDS
        ${proto_file}
    COMMENT "Generating gRPC sources from ${proto_file}"
    VERBATIM
)

target_sources(swarmkit_proto PRIVATE
    ${proto_src}
    ${grpc_src}
)

target_include_directories(swarmkit_proto PUBLIC
    ${generated_dir}
)

target_link_libraries(swarmkit_proto PUBLIC
    protobuf::libprotobuf
    gRPC::grpc++
)

swarmkit_set_warnings(swarmkit_proto)
swarmkit_enable_sanitizers(swarmkit_proto)
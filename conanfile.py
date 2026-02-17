from conan import ConanFile
from conan.tools.cmake import cmake_layout


class SwarmkitConan(ConanFile):
    name = "swarmkit"
    version = "0.2.0"
    package_type = "application"

    settings = "os", "arch", "compiler", "build_type"

    # Keep dependencies minimal & portable.
    requires = (
        "grpc/1.67.1",
        "protobuf/5.27.0",
        "spdlog/1.17.0",
        "yaml-cpp/0.8.0",
        "catch2/3.8.0",
    )

    generators = ("CMakeToolchain", "CMakeDeps", "VirtualRunEnv")

    def layout(self):
        cmake_layout(self)

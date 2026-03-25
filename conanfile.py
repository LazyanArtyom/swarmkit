# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

from pathlib import Path

from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout


class SwarmkitConan(ConanFile):
    name = "swarmkit"
    package_type = "library"

    settings = "os", "arch", "compiler", "build_type"
    options = {"shared": [True, False], "with_tools": [True, False], "with_tests": [True, False]}
    default_options = {"shared": False, "with_tools": False, "with_tests": False}

    requires = (
        "grpc/1.67.1",
        "protobuf/5.27.0",
        "spdlog/1.17.0",
        "catch2/3.8.0",
    )

    generators = ("CMakeToolchain", "CMakeDeps", "VirtualRunEnv")

    def set_version(self):
        self.version = (Path(self.recipe_folder) / "VERSION").read_text(encoding="utf-8").strip()

    def layout(self):
        cmake_layout(self)

    def build(self):
        cmake = CMake(self)
        cmake.configure(
            variables={
                "SWARMKIT_BUILD_TOOLS": "ON" if self.options.with_tools else "OFF",
                "SWARMKIT_BUILD_TESTS": "ON" if self.options.with_tests else "OFF",
                "BUILD_TESTING": "ON" if self.options.with_tests else "OFF",
            }
        )
        cmake.build()
        if self.options.with_tests:
            cmake.test()

    def package(self):
        cmake = CMake(self)
        cmake.install()

    def package_info(self):
        self.cpp_info.set_property("cmake_file_name", "SwarmKit")

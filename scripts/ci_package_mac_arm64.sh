#!/usr/bin/env bash
# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
#
# macOS ARM64 CI pipeline: conan install → build → test → sdk + tools tarballs.
# Run from the project root.
set -euo pipefail

preset="mac-release"
platform_tag="mac-arm64"

version="$(tr -d ' \t\r\n' < VERSION)"
build_dir="build/${preset}"
generators_dir="build/conan/build/Release/generators"
dist_dir="dist"

mkdir -p "${dist_dir}"

# ---------------------------------------------------------------------------
# 1) Conan: generate toolchain + cmake find-modules.
#    cmake_layout puts generators at build/conan/build/Release/generators/.
# ---------------------------------------------------------------------------
conan install . \
    -of build/conan \
    -s build_type=Release \
    -s compiler.cppstd=23 \
    --build=missing

# ---------------------------------------------------------------------------
# 2) Configure + build + test
# ---------------------------------------------------------------------------
cmake --preset "${preset}"
cmake --build --preset "${preset}"
ctest --preset "${preset}" --output-on-failure

# ---------------------------------------------------------------------------
# bundle_cmake_deps <stage_root>
#
# Copies Conan-generated CMakeDeps find files into
# <stage_root>/lib/cmake/SwarmKit/deps/ and patches the *-data.cmake files
# so the absolute staging path is replaced with ${_swarmkit_tp}, making the
# cmake find files relocatable when the SDK tarball is unpacked anywhere.
# SwarmKitConfig.cmake sets _swarmkit_tp from PACKAGE_PREFIX_DIR at configure time.
# ---------------------------------------------------------------------------
bundle_cmake_deps() {
    local stage_root="$1"
    local dest_dir="${stage_root}/lib/cmake/SwarmKit/deps"

    local tp_abs
    tp_abs="$(cd "${stage_root}" && pwd)/third_party/full_deploy/host"

    mkdir -p "${dest_dir}"

    for cmake_file in "${generators_dir}"/*.cmake; do
        local fname
        fname="$(basename "${cmake_file}")"

        case "${fname}" in
            conan_toolchain.cmake|conandeps_legacy.cmake) continue ;;
        esac

        if [[ "${fname}" == *-data.cmake ]]; then
            sed "s|${tp_abs}|\${_swarmkit_tp}|g" "${cmake_file}" > "${dest_dir}/${fname}"
        else
            cp "${cmake_file}" "${dest_dir}/${fname}"
        fi
    done

    local count
    count="$(ls "${dest_dir}"/*.cmake 2>/dev/null | wc -l | tr -d ' ')"
    echo "  Bundled ${count} cmake dep files → ${dest_dir}"
}

# ---------------------------------------------------------------------------
# 3) Stage and pack each CPack component
# ---------------------------------------------------------------------------
package_component() {
    local component="$1"
    local base="swarmkit-${version}-${component}-${platform_tag}"
    local stage_root="${build_dir}/stage/${base}"
    local out="${dist_dir}/${base}.tar.gz"

    rm -rf "${stage_root}"
    mkdir -p "${stage_root}"

    cmake --install "${build_dir}" --prefix "${stage_root}" --component "${component}"

    if [[ "${component}" == "sdk" ]]; then
        # Deploy bundled deps. Reuse -of build/conan (same as build step) so
        # Conan does not add a new entry to CMakeUserPresets.json (which would
        # create a duplicate conan-release preset and break cmake).
        conan install . \
            -of build/conan \
            -s build_type=Release \
            -s compiler.cppstd=23 \
            --build=missing \
            --deployer=full_deploy \
            --deployer-folder "${stage_root}/third_party"

        if ! ls "${stage_root}/lib/"libswarmkit_*.a >/dev/null 2>&1; then
            echo "ERROR: SDK install produced no SwarmKit libs under ${stage_root}/lib/" >&2
            exit 2
        fi

        if [[ ! -d "${stage_root}/third_party/full_deploy" ]]; then
            echo "ERROR: full_deploy did not create ${stage_root}/third_party/full_deploy/" >&2
            exit 2
        fi

        bundle_cmake_deps "${stage_root}"
    fi

    tar -czf "${out}" -C "${build_dir}/stage" "${base}"
    echo "Created: ${out}"
}

package_component sdk
package_component tools

echo ""
echo "Artifacts in ${dist_dir}/:"
ls -1 "${dist_dir}" | grep "${platform_tag}" || true

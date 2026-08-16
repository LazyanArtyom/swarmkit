#!/usr/bin/env bash
# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
#
# Build, test, stage, and package SwarmKit SDK/tools tarballs.
#
# Usage:
#   ./scripts/ci_package.sh
#   ./scripts/ci_package.sh --preset mac-release --platform mac-arm64
#   ./scripts/ci_package.sh --preset linux-release --platform linux-x86_64
set -euo pipefail

preset=""
platform_tag=""
dist_dir="dist"

usage() {
    cat >&2 <<'EOF'
Usage: scripts/ci_package.sh [--preset PRESET] [--platform PLATFORM] [--dist DIR]

Examples:
  scripts/ci_package.sh
  scripts/ci_package.sh --preset mac-release --platform mac-arm64
  scripts/ci_package.sh --preset linux-release --platform linux-x86_64

When --preset and --platform are omitted, the script auto-detects the host:
  macOS ARM64    -> --preset mac-release --platform mac-arm64
  Linux x86_64   -> --preset linux-release --platform linux-x86_64
EOF
}

detect_host_platform() {
    local os
    local arch

    os="$(uname -s)"
    arch="$(uname -m)"

    case "${os}:${arch}" in
        Darwin:arm64|Darwin:aarch64)
            printf '%s\n' "mac-arm64"
            ;;
        Linux:x86_64|Linux:amd64)
            printf '%s\n' "linux-x86_64"
            ;;
        *)
            echo "ERROR: unsupported host platform ${os}/${arch}; pass --preset and --platform explicitly." >&2
            return 1
            ;;
    esac
}

default_preset_for_platform() {
    case "$1" in
        mac-arm64)
            printf '%s\n' "mac-release"
            ;;
        linux-x86_64)
            printf '%s\n' "linux-release"
            ;;
        *)
            echo "ERROR: no default release preset for platform '$1'; pass --preset explicitly." >&2
            return 1
            ;;
    esac
}

default_platform_for_preset() {
    case "$1" in
        mac-release)
            printf '%s\n' "mac-arm64"
            ;;
        linux-release)
            printf '%s\n' "linux-x86_64"
            ;;
        *)
            return 1
            ;;
    esac
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preset)
            preset="${2:-}"
            shift 2
            ;;
        --platform)
            platform_tag="${2:-}"
            shift 2
            ;;
        --dist)
            dist_dir="${2:-}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $1" >&2
            usage
            exit 2
            ;;
    esac
done

if [[ -z "${platform_tag}" ]]; then
    if [[ -n "${preset}" ]]; then
        platform_tag="$(default_platform_for_preset "${preset}")" || {
            echo "ERROR: no default platform for preset '${preset}'; pass --platform explicitly." >&2
            exit 2
        }
    else
        platform_tag="$(detect_host_platform)" || exit 2
    fi
fi

if [[ -z "${preset}" ]]; then
    preset="$(default_preset_for_platform "${platform_tag}")" || exit 2
fi

version="$(tr -d ' \t\r\n' < VERSION)"
build_type="Release"
build_dir="build/${preset}"
conan_root="build/conan"
generators_dir="${conan_root}/build/${build_type}/generators"

mkdir -p "${dist_dir}"

conan_install() {
    local with_tests="$1"
    shift
    rm -rf "${generators_dir}"
    conan install . \
        -of "${conan_root}" \
        -s build_type="${build_type}" \
        -s compiler.cppstd=23 \
        -o "&:with_tests=${with_tests}" \
        -o "&:with_tools=True" \
        --build=missing "$@"
}

bundle_cmake_deps() {
    local stage_root="$1"
    local source_generators_dir="$2"
    local dest_dir="${stage_root}/lib/cmake/SwarmKit/deps"
    local tp_abs
    tp_abs="$(cd "${stage_root}" && pwd)/third_party/full_deploy/host"

    mkdir -p "${dest_dir}"

    for cmake_file in "${source_generators_dir}"/*.cmake; do
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
    count="$(find "${dest_dir}" -maxdepth 1 -name '*.cmake' -type f | wc -l | tr -d ' ')"
    echo "  Bundled ${count} CMake dependency files -> ${dest_dir}"
}

package_component() {
    local component="$1"
    local base="swarmkit-${version}-${component}-${platform_tag}"
    local stage_root="${build_dir}/stage/${base}"
    local out="${dist_dir}/${base}.tar.gz"

    rm -rf "${stage_root}"
    mkdir -p "${stage_root}"

    cmake --install "${build_dir}" --prefix "${stage_root}" --component "${component}"

    if [[ "${component}" == "sdk" ]]; then
        local deploy_conan_root="${stage_root}/.conan-sdk"
        local deploy_generators_dir="${deploy_conan_root}/build/${build_type}/generators"
        conan install . \
            -of "${deploy_conan_root}" \
            -s build_type="${build_type}" \
            -s compiler.cppstd=23 \
            -o "&:with_tests=False" \
            -o "&:with_tools=True" \
            -c 'tools.cmake.cmaketoolchain:user_presets=' \
            --build=missing \
            --deployer=full_deploy \
            --deployer-folder "${stage_root}/third_party"

        if ! find "${stage_root}/lib" -maxdepth 1 -name 'libswarmkit_*.a' -type f | grep -q .; then
            echo "ERROR: SDK install produced no SwarmKit libs under ${stage_root}/lib/" >&2
            exit 2
        fi

        if [[ ! -d "${stage_root}/third_party/full_deploy" ]]; then
            echo "ERROR: full_deploy did not create ${stage_root}/third_party/full_deploy/" >&2
            exit 2
        fi

        bundle_cmake_deps "${stage_root}" "${deploy_generators_dir}"
        rm -rf "${deploy_conan_root}"
    elif [[ "${component}" == "tools" ]]; then
        for tool in swarmkit-agent swarmkit-cli swarmkit-evidence-inspect; do
            if [[ ! -x "${stage_root}/bin/${tool}" ]]; then
                echo "ERROR: tools install is missing executable ${tool}" >&2
                exit 2
            fi
        done
    fi

    tar -czf "${out}" -C "${build_dir}/stage" "${base}"
    echo "Created: ${out}"
}

write_checksum() {
    local archive="$1"
    local archive_dir
    local archive_name
    archive_dir="$(dirname "${archive}")"
    archive_name="$(basename "${archive}")"

    if command -v sha256sum >/dev/null 2>&1; then
        (cd "${archive_dir}" && sha256sum "${archive_name}" > "${archive_name}.sha256")
    elif command -v shasum >/dev/null 2>&1; then
        (cd "${archive_dir}" && shasum -a 256 "${archive_name}" > "${archive_name}.sha256")
    else
        echo "ERROR: sha256sum or shasum is required to checksum release archives" >&2
        exit 2
    fi
}

conan_install True
cmake --preset "${preset}"
cmake --build --preset "${preset}"
ctest --preset "${preset}" --output-on-failure

package_component sdk
package_component tools
write_checksum "${dist_dir}/swarmkit-${version}-sdk-${platform_tag}.tar.gz"
write_checksum "${dist_dir}/swarmkit-${version}-tools-${platform_tag}.tar.gz"

echo ""
echo "Artifacts in ${dist_dir}:"
find "${dist_dir}" -maxdepth 1 -name "*${platform_tag}*.tar.gz*" -type f -print | sort

#!/usr/bin/env bash
# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
#
# Deploy an SDK tarball to a local prefix for external CMake consumers.
set -euo pipefail

prefix="${SWARMKIT_SDK_PREFIX:-${HOME}/swarmkit-sdk}"
dist_dir="dist"
platform_tag=""
tarball=""

usage() {
    cat >&2 <<'EOF'
Usage: scripts/deploy_sdk.sh [--prefix DIR] [--platform TAG] [TARBALL]

Examples:
  scripts/deploy_sdk.sh --platform mac-arm64 --prefix /tmp/swarmkit-sdk
  scripts/deploy_sdk.sh dist/swarmkit-0.1.0-sdk-mac-arm64.tar.gz

Environment:
  SWARMKIT_SDK_PREFIX  Default install prefix. Defaults to ~/swarmkit-sdk.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix)
            prefix="${2:-}"
            shift 2
            ;;
        --platform)
            platform_tag="${2:-}"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "ERROR: unknown argument: $1" >&2
            usage
            exit 2
            ;;
        *)
            if [[ -n "${tarball}" ]]; then
                echo "ERROR: only one tarball path may be provided" >&2
                exit 2
            fi
            tarball="$1"
            shift
            ;;
    esac
done

if [[ -z "${prefix}" ]]; then
    echo "ERROR: --prefix cannot be empty" >&2
    exit 2
fi

if [[ -z "${tarball}" ]]; then
    if [[ -n "${platform_tag}" ]]; then
        tarball="$(find "${dist_dir}" -maxdepth 1 -name "swarmkit-*-sdk-${platform_tag}.tar.gz" -type f -print | sort | tail -1)"
    else
        tarball="$(find "${dist_dir}" -maxdepth 1 -name "swarmkit-*-sdk-*.tar.gz" -type f -print | sort | tail -1)"
    fi
fi

if [[ -z "${tarball}" || ! -f "${tarball}" ]]; then
    echo "ERROR: SDK tarball not found. Build one with scripts/ci_package_*.sh first." >&2
    exit 1
fi

echo "Deploying SDK: ${tarball}"
echo "Prefix:        ${prefix}"

rm -rf "${prefix:?}"/*
mkdir -p "${prefix}"
tar xzf "${tarball}" -C "${prefix}" --strip-components=1

if [[ ! -f "${prefix}/lib/cmake/SwarmKit/SwarmKitConfig.cmake" ]]; then
    echo "ERROR: deployed SDK is missing lib/cmake/SwarmKit/SwarmKitConfig.cmake" >&2
    exit 2
fi

cat <<EOF
Done. SDK installed at ${prefix}

Build an external consumer with:
  cmake -S apps/test_tools -B /tmp/swarmkit-sdk-probe-build -DCMAKE_PREFIX_PATH=${prefix} -DCMAKE_BUILD_TYPE=Release
  cmake --build /tmp/swarmkit-sdk-probe-build
  /tmp/swarmkit-sdk-probe-build/swarmkit-sdk-ping --addr 127.0.0.1:50061 --insecure
EOF

#!/usr/bin/env bash
# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
#
# Deploy the packaged SDK tarball to ~/swarmkit-sdk.
# Clears the target directory and extracts the latest build.
#
# Usage:
#   ./scripts/deploy_sdk_mac.sh              (auto-detect tarball in dist/)
#   ./scripts/deploy_sdk_mac.sh path/to.tar.gz  (explicit tarball)
set -euo pipefail

sdk_dir="${HOME}/swarmkit-sdk"
dist_dir="dist"

if [[ $# -ge 1 ]]; then
    tarball="$1"
else
    tarball="$(ls -t "${dist_dir}"/swarmkit-*-sdk-mac-arm64.tar.gz 2>/dev/null | head -1)"
    if [[ -z "${tarball}" ]]; then
        echo "ERROR: no SDK tarball found in ${dist_dir}/" >&2
        exit 1
    fi
fi

if [[ ! -f "${tarball}" ]]; then
    echo "ERROR: tarball not found: ${tarball}" >&2
    exit 1
fi

echo "Deploying SDK: ${tarball}"
echo "Target:        ${sdk_dir}"

rm -rf "${sdk_dir:?}"/*
mkdir -p "${sdk_dir}"
tar xzf "${tarball}" -C "${sdk_dir}" --strip-components=1

echo "Done. SDK installed at ${sdk_dir}"
ls "${sdk_dir}"

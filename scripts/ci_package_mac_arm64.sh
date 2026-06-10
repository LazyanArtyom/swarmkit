#!/usr/bin/env bash
# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
exec ./scripts/ci_package.sh --preset mac-release --platform mac-arm64 "$@"

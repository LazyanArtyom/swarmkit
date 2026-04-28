# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.

include_guard(GLOBAL)

function(swarmkit_set_warnings target_name)
    target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
endfunction()

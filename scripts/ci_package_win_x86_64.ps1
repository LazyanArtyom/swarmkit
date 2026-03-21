# Copyright (c) 2026 Artyom Lazyan. All rights reserved.
# SPDX-License-Identifier: LicenseRef-SwarmKit-Proprietary
#
# This file is part of SwarmKit.
# See LICENSE.md in the repository root for full license terms.
#
# Windows x86_64 CI pipeline: conan install -> build -> test -> sdk + tools zips.
# Run from the project root.
#
#   powershell -ExecutionPolicy Bypass -File scripts\ci_package_win_x86_64.ps1

param(
    [string]$Preset = "win-release"
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$PlatformTag  = "win-x86_64"
$Version      = (Get-Content VERSION -Raw).Trim()
$BuildDir     = "build\$Preset"
$GenDir       = "build\conan\build\Release\generators"
$DistDir      = "dist"

New-Item -ItemType Directory -Force -Path $DistDir | Out-Null

# ---------------------------------------------------------------------------
# 1) Conan: generate toolchain + cmake find-modules.
# ---------------------------------------------------------------------------
conan install . `
    -of build\conan `
    -s build_type=Release `
    -s compiler.cppstd=23 `
    --build=missing

# ---------------------------------------------------------------------------
# 2) Configure + build + test
# ---------------------------------------------------------------------------
cmake --preset $Preset
cmake --build --preset $Preset
ctest --preset $Preset --output-on-failure

# ---------------------------------------------------------------------------
# Bundle-CmakeDeps <StageRoot>
#
# Copies Conan-generated CMakeDeps find files into
# <StageRoot>\lib\cmake\SwarmKit\deps\ and patches *-data.cmake files so the
# absolute staging path is replaced with ${_swarmkit_tp}, making the cmake
# find files relocatable when the SDK zip is unpacked anywhere.
# ---------------------------------------------------------------------------
function Bundle-CmakeDeps([string]$StageRoot) {
    $DestDir = Join-Path $StageRoot "lib\cmake\SwarmKit\deps"
    $TpAbs   = (Resolve-Path $StageRoot).Path + "\third_party\full_deploy\host"

    New-Item -ItemType Directory -Force -Path $DestDir | Out-Null

    $Count = 0
    Get-ChildItem "$GenDir\*.cmake" | ForEach-Object {
        $FName = $_.Name
        if ($FName -eq "conan_toolchain.cmake" -or $FName -eq "conandeps_legacy.cmake") { return }

        $Dest = Join-Path $DestDir $FName
        if ($FName -like "*-data.cmake") {
            (Get-Content $_.FullName -Raw) `
                -replace [regex]::Escape($TpAbs), '${_swarmkit_tp}' |
                Set-Content -NoNewline -Path $Dest
        } else {
            Copy-Item $_.FullName $Dest
        }
        $Count++
    }
    Write-Host "  Bundled $Count cmake dep files -> $DestDir"
}

# ---------------------------------------------------------------------------
# 3) Stage and pack each component
# ---------------------------------------------------------------------------
function Package-Component([string]$Component) {
    $Base      = "swarmkit-$Version-$Component-$PlatformTag"
    $StageRoot = "$BuildDir\stage\$Base"
    $Out       = "$DistDir\$Base.zip"

    if (Test-Path $StageRoot) { Remove-Item -Recurse -Force $StageRoot }
    New-Item -ItemType Directory -Force -Path $StageRoot | Out-Null

    cmake --install $BuildDir --prefix $StageRoot --component $Component

    if ($Component -eq "sdk") {
        conan install . `
            -of build\conan `
            -s build_type=Release `
            -s compiler.cppstd=23 `
            --build=missing `
            --deployer=full_deploy `
            "--deployer-folder=$StageRoot\third_party"

        $Libs = Get-ChildItem "$StageRoot\lib\swarmkit_*.lib" -ErrorAction SilentlyContinue
        if ($null -eq $Libs -or $Libs.Count -eq 0) {
            throw "ERROR: SDK install produced no SwarmKit libs under $StageRoot\lib\"
        }

        if (-not (Test-Path "$StageRoot\third_party\full_deploy")) {
            throw "ERROR: full_deploy did not create $StageRoot\third_party\full_deploy\"
        }

        Bundle-CmakeDeps $StageRoot
    }

    if (Test-Path $Out) { Remove-Item -Force $Out }

    # Compress-Archive from the stage parent so the zip root is the base folder.
    Push-Location "$BuildDir\stage"
    try {
        Compress-Archive -Path $Base -DestinationPath (Join-Path (Resolve-Path "..\..\..") $Out)
    } finally {
        Pop-Location
    }
    Write-Host "Created: $Out"
}

Package-Component "sdk"
Package-Component "tools"

Write-Host ""
Write-Host "Artifacts in ${DistDir}:"
Get-ChildItem $DistDir |
    Where-Object { $_.Name -like "*$PlatformTag*" } |
    ForEach-Object { $_.Name }

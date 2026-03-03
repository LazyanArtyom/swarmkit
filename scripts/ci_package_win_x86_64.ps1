param(
  [string]$Preset = "win-release"
)

# Windows runner (x86_64)
# Produces:
#   dist\swarmkit-<ver>-sdk-win-x86_64.zip
#   dist\swarmkit-<ver>-tools-win-x86_64.zip
# Both extract into a top folder with same base name.

$PlatformTag = "win-x86_64"
$Version = (Get-Content VERSION -Raw).Trim()

$BuildDir = Join-Path "build" $Preset
$PackagesDir = Join-Path $BuildDir "packages"
$DistDir = "dist"

New-Item -ItemType Directory -Force -Path $DistDir | Out-Null
New-Item -ItemType Directory -Force -Path $PackagesDir | Out-Null

# Keep repo root clean from previous cpack runs
if (Test-Path "_CPack_Packages") { Remove-Item -Recurse -Force "_CPack_Packages" }

# 1) Conan deps (Release, C++23)
conan install . -of build\conan -s build_type=Release -s compiler.cppstd=23 --build=missing

# 2) Configure + build + test
cmake --preset $Preset
cmake --build --preset $Preset
ctest --preset $Preset --output-on-failure

function Repack-Zip-WithTopDir([string]$InputZip, [string]$TopDirName, [string]$OutputZip) {
  $tmp = Join-Path ([System.IO.Path]::GetTempPath()) ([System.Guid]::NewGuid().ToString())
  $inDir = Join-Path $tmp "in"
  $outDir = Join-Path $tmp "out"
  $topDir = Join-Path $outDir $TopDirName

  New-Item -ItemType Directory -Force -Path $inDir | Out-Null
  New-Item -ItemType Directory -Force -Path $topDir | Out-Null

  Expand-Archive -Force -Path $InputZip -DestinationPath $inDir

  Get-ChildItem -Force $inDir | ForEach-Object {
    Move-Item -Force $_.FullName $topDir
  }

  if (Test-Path $OutputZip) { Remove-Item -Force $OutputZip }
  Compress-Archive -Path $topDir -DestinationPath $OutputZip

  Remove-Item -Recurse -Force $tmp
}

function Run-Component([string]$Component) {
  $Base = "swarmkit-$Version-$Component-$PlatformTag"
  $Out = Join-Path $DistDir "$Base.zip"

  # Remove only old zip outputs in the build packages dir to avoid stale picks
  Remove-Item -Force "$PackagesDir\*.zip" -ErrorAction SilentlyContinue

  # Generate component archive into build\<preset>\packages\
  cpack --config "$BuildDir\CPackConfig.cmake" -G ZIP -D "CPACK_COMPONENTS_ALL=$Component"

  $Produced = Get-ChildItem "$PackagesDir\*.zip" -ErrorAction SilentlyContinue |
              Sort-Object LastWriteTime -Descending | Select-Object -First 1
  if ($null -eq $Produced) {
    throw "ERROR: cpack produced no .zip in $PackagesDir"
  }

  Repack-Zip-WithTopDir -InputZip $Produced.FullName -TopDirName $Base -OutputZip $Out
  Write-Host "Created: $Out"
}

Run-Component "sdk"
Run-Component "tools"

# Remove cpack temp folder in repo root if it appeared
if (Test-Path "_CPack_Packages") { Remove-Item -Recurse -Force "_CPack_Packages" }

Write-Host "Artifacts:"
Get-ChildItem $DistDir | Where-Object { $_.Name -like "*$PlatformTag*" } | ForEach-Object { $_.Name }
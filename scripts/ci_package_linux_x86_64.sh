#!/usr/bin/env bash
set -euo pipefail

# Linux runner (x86_64)
# Produces:
#   dist/swarmkit-<ver>-sdk-linux-x86_64.tar.gz
#   dist/swarmkit-<ver>-tools-linux-x86_64.tar.gz
# Both extract into a top folder with same base name.

preset="linux-release"
platform_tag="linux-x86_64"

version="$(tr -d ' \t\r\n' < VERSION)"
build_dir="build/${preset}"
packages_dir="${build_dir}/packages"
dist_dir="dist"

mkdir -p "${dist_dir}"
mkdir -p "${packages_dir}"

# Keep repo root clean from previous cpack runs
rm -rf "_CPack_Packages" 2>/dev/null || true

# 1) Conan deps (Release, C++23)
conan install . -of build/conan -s build_type=Release -s compiler.cppstd=23 --build=missing

# 2) Configure + build + test
cmake --preset "${preset}"
cmake --build --preset "${preset}"
ctest --preset "${preset}" --output-on-failure

# Helper: repack tgz to include a single top folder
repack_tgz_with_topdir() {
  local input_tgz="$1"
  local topdir="$2"
  local output_tgz="$3"

  local tmp_dir
  tmp_dir="$(mktemp -d)"

  # Trap uses concrete path (no unbound-variable issues with set -u)
  trap "rm -rf '${tmp_dir}'" RETURN

  mkdir -p "${tmp_dir}/in" "${tmp_dir}/out/${topdir}"

  tar -xzf "${input_tgz}" -C "${tmp_dir}/in"
  shopt -s dotglob nullglob
  mv "${tmp_dir}/in"/* "${tmp_dir}/out/${topdir}/"
  tar -czf "${output_tgz}" -C "${tmp_dir}/out" "${topdir}"
}

run_component() {
  local component="$1"
  local base="swarmkit-${version}-${component}-${platform_tag}"
  local out="${dist_dir}/${base}.tar.gz"

  # Remove only old tgz outputs in the build packages dir to avoid stale picks
  rm -f "${packages_dir}/"*.tar.gz 2>/dev/null || true

  # Generate component archive into build/<preset>/packages/
  cpack --config "${build_dir}/CPackConfig.cmake" -G TGZ -D "CPACK_COMPONENTS_ALL=${component}"

  # Find newest archive produced
  local produced
  produced="$(ls -1t "${packages_dir}/"*.tar.gz 2>/dev/null | head -n 1 || true)"
  if [[ -z "${produced}" ]]; then
    echo "ERROR: cpack produced no .tar.gz in ${packages_dir}"
    exit 2
  fi

  repack_tgz_with_topdir "${produced}" "${base}" "${out}"
  echo "Created: ${out}"
}

run_component sdk
run_component tools

# Remove cpack temp folder in repo root if it appeared
rm -rf "_CPack_Packages" 2>/dev/null || true

echo "Artifacts:"
ls -1 "${dist_dir}" | grep "${platform_tag}" || true
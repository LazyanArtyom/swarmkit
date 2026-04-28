#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
script_path="${script_dir}/run-clang-tidy.sh"
tidy_compile_db_to_remove=""

resolve_path() {
    local candidate="$1"

    if [[ "${candidate}" = /* ]]; then
        printf '%s\n' "${candidate}"
        return
    fi

    printf '%s/%s\n' "${workspace_dir}" "${candidate#./}"
}

escape_regex() {
    printf '%s' "$1" | sed 's/[][(){}.^$*+?|\\]/\\&/g'
}

cleanup() {
    if [[ -n "${tidy_compile_db_to_remove}" ]]; then
        rm -rf "${tidy_compile_db_to_remove}"
    fi
}

filter_project_diagnostics() {
    local escaped_workspace_dir
    local project_file_regex

    escaped_workspace_dir="$(escape_regex "${workspace_dir}/")"
    project_file_regex="^${escaped_workspace_dir}(apps|include|src|tests)/.*:[0-9]+:[0-9]+: (warning|error):"

    awk -v project_file_regex="${project_file_regex}" '
        function flush_block() {
            if (in_block && keep_block) {
                printf "%s", block
            }
            block = ""
            in_block = 0
            keep_block = 1
        }

        /^[^[:space:]].*:[0-9]+:[0-9]+: (warning|error):/ {
            flush_block()
            in_block = 1
            keep_block = ($0 ~ project_file_regex)
            block = $0 "\n"
            next
        }

        {
            if (in_block) {
                block = block $0 "\n"
                next
            }
            print
        }

        END {
            flush_block()
        }
    '
}

run_single_file() {
    local build_dir="$1"
    local file_path="$2"
    local escaped_workspace_dir
    local header_filter

    escaped_workspace_dir="$(escape_regex "${workspace_dir}/")"
    header_filter="^${escaped_workspace_dir}(apps|include|src|tests)/"

    clang-tidy \
        "${file_path}" \
        -p "${build_dir}" \
        "--config-file=${workspace_dir}/.clang-tidy" \
        "--header-filter=${header_filter}" \
        2>&1 \
        | sed -E '/^[0-9]+ warnings generated\.$/d; /^Suppressed [0-9]+ warnings .*$/d; /^Use -header-filter=.*$/d' \
        | filter_project_diagnostics
}

make_tidy_compile_database() {
    local build_dir="$1"
    local compile_commands="$2"
    local temp_dir

    if ! grep -Eq -- '-fmodules-ts|-fmodule-mapper=|-fdeps-format=' "${compile_commands}"; then
        printf '%s\n' "${build_dir}"
        return
    fi

    if ! command -v python3 >/dev/null 2>&1; then
        printf 'clang-tidy: python3 is required to sanitize %s for clang-tidy\n' "${compile_commands}" >&2
        exit 1
    fi

    temp_dir="$(mktemp -d "${TMPDIR:-/tmp}/swarmkit-clang-tidy-db.XXXXXX")"
    python3 - "${compile_commands}" "${temp_dir}/compile_commands.json" <<'PY'
import json
import shlex
import sys

unsupported_prefixes = (
    "-fmodule-mapper=",
    "-fdeps-format=",
)
unsupported_with_value = {
    "-fmodule-mapper",
    "-fdeps-format",
}
unsupported_exact = {
    "-fmodules-ts",
}


def sanitized_args(args):
    result = []
    skip_next = False
    for arg in args:
        if skip_next:
            skip_next = False
            continue
        if arg in unsupported_exact:
            continue
        if arg in unsupported_with_value:
            skip_next = True
            continue
        if arg.startswith(unsupported_prefixes):
            continue
        result.append(arg)
    return result


with open(sys.argv[1], "r", encoding="utf-8") as src:
    database = json.load(src)

for entry in database:
    if "arguments" in entry:
        entry["arguments"] = sanitized_args(entry["arguments"])
    if "command" in entry:
        entry["command"] = shlex.join(sanitized_args(shlex.split(entry["command"])))

with open(sys.argv[2], "w", encoding="utf-8") as dst:
    json.dump(database, dst, indent=2)
    dst.write("\n")
PY
    printf '%s\n' "${temp_dir}"
}

collect_project_files() {
    local build_dir="$1"
    local compile_commands="$2"

    sed -nE 's/^[[:space:]]*"file"[[:space:]]*:[[:space:]]*"([^"]+)".*/\1/p' "${compile_commands}" \
        | while IFS= read -r file_path; do
            [[ -n "${file_path}" ]] || continue
            [[ "${file_path}" == "${workspace_dir}/"* ]] || continue
            [[ "${file_path}" == "${build_dir}/"* ]] && continue
            [[ "${file_path}" =~ \.(c|cc|cpp|cxx|m|mm)$ ]] || continue
            printf '%s\n' "${file_path}"
        done \
        | sort -u
}

resolve_input_file() {
    local requested_path="$1"
    local -a candidate_extensions=(".cpp" ".cc" ".cxx" ".c" ".mm" ".m")
    local base_path
    local candidate_path

    if [[ "${requested_path}" =~ \.(c|cc|cpp|cxx|m|mm)$ ]]; then
        printf '%s\n' "${requested_path}"
        return
    fi

    base_path="${requested_path%.*}"
    for extension in "${candidate_extensions[@]}"; do
        candidate_path="${base_path}${extension}"
        if [[ -f "${candidate_path}" ]]; then
            printf '%s\n' "${candidate_path}"
            return
        fi
    done

    printf '%s\n' "${requested_path}"
}

main() {
    local mode="project"
    local build_dir_arg="${1:-}"
    local build_dir
    local compile_commands
    local tidy_build_dir
    local jobs
    local -a requested_files=()
    local -a files=()

    if [[ "${build_dir_arg}" == "--single-file" ]]; then
        mode="single-file"
        shift
        build_dir_arg="${1:-}"
        shift || true
    else
        shift || true
    fi

    if [[ -z "${build_dir_arg}" ]]; then
        printf 'Usage: %s [--single-file] <build-dir> [files...]\n' "${script_path}" >&2
        exit 1
    fi

    build_dir="$(resolve_path "${build_dir_arg}")"
    compile_commands="${build_dir}/compile_commands.json"

    if [[ ! -f "${compile_commands}" ]]; then
        printf 'clang-tidy: compile database not found at %s\n' "${compile_commands}" >&2
        printf 'Run the matching configure/build task first.\n' >&2
        exit 1
    fi

    tidy_build_dir="$(make_tidy_compile_database "${build_dir}" "${compile_commands}")"
    if [[ "${tidy_build_dir}" != "${build_dir}" ]]; then
        tidy_compile_db_to_remove="${tidy_build_dir}"
        trap cleanup EXIT
    fi

    while (($# > 0)); do
        requested_files+=("$(resolve_input_file "$(resolve_path "$1")")")
        shift
    done

    if [[ "${mode}" == "single-file" ]]; then
        if ((${#requested_files[@]} != 1)); then
            printf 'clang-tidy: expected exactly one file in --single-file mode\n' >&2
            exit 1
        fi

        run_single_file "${tidy_build_dir}" "${requested_files[0]}"
        exit $?
    fi

    if ((${#requested_files[@]} > 0)); then
        files=("${requested_files[@]}")
    else
        while IFS= read -r file_path; do
            files+=("${file_path}")
        done < <(collect_project_files "${build_dir}" "${compile_commands}")
    fi

    if ((${#files[@]} == 0)); then
        printf 'clang-tidy: no project source files found in %s\n' "${compile_commands}" >&2
        exit 1
    fi

    jobs="$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || printf '4')"

    printf '%s\n' "clang-tidy: checking ${#files[@]} file(s) from ${build_dir}"
    printf '%s\0' "${files[@]}" \
        | xargs -0 -n 1 -P "${jobs}" bash "${script_path}" --single-file "${tidy_build_dir}"
}

main "$@"

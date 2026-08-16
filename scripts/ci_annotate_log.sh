#!/usr/bin/env bash

set -euo pipefail

if (($# < 2 || $# > 3)); then
    printf 'Usage: %s LOG_PATH ANNOTATION_TITLE [START_PATTERN]\n' "$0" >&2
    exit 2
fi

log_path="$1"
annotation_title="$2"
start_pattern="${3:-ERROR: AddressSanitizer|WARNING: ThreadSanitizer|runtime error:|FAILED:|error:}"

if [[ ! -f "${log_path}" ]]; then
    exit 0
fi

matched_line="$(grep -n -m 1 -E "${start_pattern}" "${log_path}" || true)"
if [[ -n "${matched_line}" ]]; then
    start_line="${matched_line%%:*}"
else
    total_lines="$(wc -l < "${log_path}")"
    start_line=$((total_lines > 200 ? total_lines - 199 : 1))
fi
end_line=$((start_line + 199))

emit_annotation() {
    local part_number="$1"
    local message="$2"

    message="${message//'%'/'%25'}"
    message="${message//$'\r'/'%0D'}"
    message="${message//$'\n'/'%0A'}"
    printf '::error title=%s (part %d)::%s\n' \
        "${annotation_title}" "${part_number}" "${message}"
}

part_number=1
line_count=0
part=""
while IFS= read -r line || [[ -n "${line}" ]]; do
    if [[ -n "${part}" ]]; then
        part+=$'\n'
    fi
    part+="${line}"
    ((line_count += 1))

    if ((line_count == 20)); then
        emit_annotation "${part_number}" "${part}"
        ((part_number += 1))
        line_count=0
        part=""
    fi
done < <(sed -n "${start_line},${end_line}p" "${log_path}")

if ((line_count > 0)); then
    emit_annotation "${part_number}" "${part}"
fi

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${1:-${SCRIPT_DIR}/../build}"

BINARIES=(
    helloworld
    add_residual_1
    add_residual_2
    multivariate
    pitch_estimation
    se3_estimation
    se3_estimation_malloc
    pass_by_address
)

PASS=0
FAIL=0

for bin in "${BINARIES[@]}"; do
    exe="${BUILD_DIR}/${bin}"
    if [ ! -f "$exe" ]; then
        echo "MISSING: $exe"
        (( FAIL += 1 ))
        continue
    fi
    echo "--- Running ${bin} ---"
    if "${exe}" > /dev/null 2>&1; then
        echo "PASS: ${bin}"
        (( PASS += 1 ))
    else
        echo "FAIL: ${bin} (exit code $?)"
        (( FAIL += 1 ))
    fi
done

echo ""
echo "==============================="
echo "Results: ${PASS} passed, ${FAIL} failed"
echo "==============================="
[ "${FAIL}" -eq 0 ]

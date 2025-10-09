#!/usr/bin/env bash
set -euo pipefail

CF_BIN=${CLANG_FORMAT_BIN:-clang-format}

if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  files=$(git ls-files '*.c' '*.cpp' '*.h' '*.hpp')
else
  files=$(find . -type f \( -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \))
fi

if [ -z "${files}" ]; then
  echo "No C/C++ source files found."
  exit 0
fi

echo "$files" | xargs -I{} "$CF_BIN" -i {}
echo "Formatting complete."
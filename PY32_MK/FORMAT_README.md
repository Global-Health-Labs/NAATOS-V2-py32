# PUYA32 HAL — C++ Core Guidelines Formatting Kit

This kit standardizes formatting and provides advisory feedback against the C++ Core Guidelines
**without changing behavior.**

## Files
- `.clang-format` — Layout-only, conservative style (LLVM base)
- `.clang-tidy` — Enables `cppcoreguidelines-*` and a few helpful checks (warnings-only)
- `tools/format.sh` — Bash/MSYS2 script to run clang-format in-place
- `tools/format.ps1` — PowerShell equivalent for Windows

## Quick Start
1) Place these files in your repo root.
2) Ensure you have `clang-format` and optionally `clang-tidy` installed.
3) Run one of:
   - `bash tools/format.sh`
   - `powershell -ExecutionPolicy Bypass -File tools/format.ps1`

## Workflow (safe)
- Commit current tree.
- Run formatter; review `git diff` (whitespace/layout only).
- Run `clang-tidy -p build -header-filter=.* --warnings-as-errors= ` to collect advisories.
- Build & smoke test on hardware.

## Notes
- Do not auto-apply clang-tidy fixes without review.
- Treat guideline findings as a backlog for focused, separate refactor PRs.
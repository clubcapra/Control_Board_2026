@echo off
REM =============================================================================
REM  run_cppcheck.bat - Static analysis launcher (R3 / DO-178C residual)
REM =============================================================================
REM
REM  Usage (from the repo root):
REM      tools\run_cppcheck.bat
REM
REM  Exit code 0 = clean (no warnings).  Non-zero = at least one issue that
REM  must be fixed before the next certification baseline is released.
REM
REM  cppcheck installation on Windows:
REM      winget install -e --id Cppcheck.Cppcheck
REM  or download from: https://cppcheck.sourceforge.io/
REM =============================================================================

where /Q cppcheck
if errorlevel 1 (
    echo [run_cppcheck] ERROR: cppcheck not found in PATH.
    echo [run_cppcheck] Install with: winget install -e --id Cppcheck.Cppcheck
    exit /b 2
)

echo [run_cppcheck] Running cppcheck on src/ and include/...
cppcheck ^
    --enable=all ^
    --inconclusive ^
    --std=c++14 ^
    --platform=unix32 ^
    --suppressions-list=tools\cppcheck.cfg ^
    --error-exitcode=1 ^
    -I include ^
    -I src ^
    src ^
    include

if errorlevel 1 (
    echo [run_cppcheck] FAILED: at least one warning emitted.
    exit /b 1
)

echo [run_cppcheck] PASSED: zero warnings on PDU CSCI.
exit /b 0

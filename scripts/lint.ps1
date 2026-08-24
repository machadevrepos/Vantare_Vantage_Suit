# Lints C/C++ sources with cppcheck (static analysis).
#
#   pwsh -File scripts\lint.ps1
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

$cppcheck = Get-Command cppcheck -ErrorAction SilentlyContinue
if ($null -eq $cppcheck) {
    Write-Error 'cppcheck not found on PATH. Install: winget install cppcheck'
    exit 2
}

& cppcheck `
    --enable=warning,performance,portability `
    --std=c++17 `
    --inline-suppr `
    --error-exitcode=1 `
    --suppress='*:*/third_party/*' `
    --suppress='*:*/STM32_WPAN/*' `
    --suppress='*:*/Middlewares/*' `
    --suppress='*:*/Drivers/*' `
    --suppress='*:*/Utilities/*' `
    -I (Join-Path $repoRoot 'firmware\common\inc') `
    (Join-Path $repoRoot 'firmware\common\src') `
    (Join-Path $repoRoot 'host\tests\cpp')
exit $LASTEXITCODE

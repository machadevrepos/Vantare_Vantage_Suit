# Formats all C/C++ sources with clang-format (if available).
#
#   pwsh -File scripts\format.ps1
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

$clangFormat = Get-Command clang-format -ErrorAction SilentlyContinue
if ($null -eq $clangFormat) {
    Write-Error 'clang-format not found on PATH. Install LLVM: winget install LLVM.LLVM'
    exit 2
}

$files = Get-ChildItem -Recurse -Path (Join-Path $repoRoot 'firmware'), (Join-Path $repoRoot 'host') `
    -Include '*.cpp', '*.h', '*.cc', '*.hpp' -File |
    Where-Object { $_.FullName -notmatch '\\(build|Debug|Release|third_party|STM32_WPAN|Middlewares|Drivers|Utilities|FATFS)\\' }

Write-Output "formatting $($files.Count) files..."
foreach ($file in $files) {
    & clang-format -i --style=file $file.FullName
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
Write-Output 'format complete.'

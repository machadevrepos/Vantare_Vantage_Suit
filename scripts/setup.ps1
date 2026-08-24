# Repo bootstrap: verify toolchain + python deps for host tests.
#
#   pwsh -File scripts\setup.ps1
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

Write-Output '== Python =='
python --version

Write-Output '== Python test deps =='
python -m pip install --quiet pytest
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Output '== CMake =='
$cmake = Get-Command cmake -ErrorAction SilentlyContinue
if ($null -eq $cmake) {
    Write-Warning 'cmake not found on PATH; install it to run the C++ host tests.'
} else {
    cmake --version | Select-Object -First 1
}

Write-Output '== C++ compiler =='
$cxx = Get-Command g++ -ErrorAction SilentlyContinue
if ($null -eq $cxx) {
    Write-Warning 'g++ not found on PATH; install MinGW/winget gcc or use MSVC + cmake generator.'
} else {
    & g++ --version | Select-Object -First 1
}

Write-Output '== STM32CubeIDE (optional, firmware builds) =='
$cube = Get-ChildItem 'C:\ST\STM32CubeIDE_*' -Directory -ErrorAction SilentlyContinue
if ($cube) { Write-Output "found: $($cube.Name)" } else { Write-Output 'not found (firmware builds unavailable on this machine)' }

Write-Output 'setup complete.'

# Runs the full host test suite on Windows: Python invariant tests + C++ module tests.
#
#   pwsh -File host\tests\run_tests.ps1          # everything
#   pwsh -File host\tests\run_tests.ps1 py       # python only
#   pwsh -File host\tests\run_tests.ps1 cpp      # c++ only
param(
    [ValidateSet('all', 'py', 'cpp')]
    [string]$Mode = 'all'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..\..')

function Invoke-Python {
    Write-Output '== Python invariant tests =='
    python -m pytest (Join-Path $repoRoot 'host\tests\python') -v
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

function Invoke-Cpp {
    Write-Output '== C++ module tests =='
    $src = Join-Path $repoRoot 'host\tests\cpp'
    $build = Join-Path $src 'build'
    cmake -S $src -B $build
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    cmake --build $build --config Release
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    ctest --test-dir $build --output-on-failure -C Release
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

switch ($Mode) {
    'py'  { Invoke-Python }
    'cpp' { Invoke-Cpp }
    'all' { Invoke-Python; Invoke-Cpp }
}

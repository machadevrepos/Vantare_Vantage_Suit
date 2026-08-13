# Compile-checks the host test suites.
#
# The constexpr/static_assert suites are evaluated by `arm-none-eabi-g++
# -fsyntax-only`; the remaining files receive an API/syntax compile guard. This
# script must fail when the compiler is unavailable rather than reporting a
# successful check that compiled nothing.
#
#   pwsh -File Firmware/HostTests/run_host_syntax_suite.ps1

$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')

$compiler = $env:ARM_NONE_EABI_GXX
if ([string]::IsNullOrWhiteSpace($compiler)) {
    $onPath = Get-Command 'arm-none-eabi-g++' -ErrorAction SilentlyContinue
    if ($null -ne $onPath) {
        $compiler = $onPath.Source
    }
}
if ([string]::IsNullOrWhiteSpace($compiler)) {
    $cubeCandidates = @(
        Get-ChildItem 'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.*\tools\bin\arm-none-eabi-g++.exe' -File -ErrorAction SilentlyContinue
    ) | Sort-Object FullName -Descending
    if ($cubeCandidates.Count -gt 0) {
        $compiler = $cubeCandidates[0].FullName
    }
}
if ([string]::IsNullOrWhiteSpace($compiler) -or -not (Test-Path $compiler)) {
    Write-Error "ARM compiler not found. Put arm-none-eabi-g++ on PATH or set ARM_NONE_EABI_GXX to the executable path."
    exit 2
}

$includes = @(
    '-I', (Join-Path $repo 'Firmware\LIBRARY\CUSTOM'),
    '-I', (Join-Path $repo 'Firmware\Master\Core\Inc'),
    '-I', (Join-Path $repo 'Firmware\HostTests\FatFsStub')
)

# Sources that compile against the FatFs stub and the CUSTOM library.
$sources = @(
    'Firmware\HostTests\test_master_node_reliable_control.cpp',
    'Firmware\HostTests\test_master_node_transfer_window.cpp',
    'Firmware\HostTests\test_hub_leaf_ble_manager.cpp',
    'Firmware\HostTests\test_node_live_sample_queue.cpp',
    'Firmware\HostTests\test_coderabbit_csv_logger_regressions.cpp',
    'Firmware\HostTests\test_coderabbit_firmware_regressions.cpp',
    'Firmware\HostTests\test_master_node_session_stager_sequential.cpp',
    'Firmware\HostTests\test_master_training_csv_formatter_v2.cpp',
    'Firmware\HostTests\test_master_training_csv_logger_v2.cpp',
    'Firmware\HostTests\test_remote_transfer_lifecycle_regression.cpp',
    'Firmware\Master\tests\master_training_csv_coordinator_test.cpp',
    'Firmware\Master\tests\master_node_session_stager_test.cpp',
    'Firmware\Master\tests\master_session_timestamp_ledger_test.cpp',
    'Firmware\Master\tests\acquisition_diagnostics_test.cpp'
)

$failed = 0
foreach ($relative in $sources) {
    $path = Join-Path $repo $relative
    if (-not (Test-Path $path)) {
        $failed++
        Write-Output "FAIL $relative (not found)"
        continue
    }
    $output = & $compiler -std=gnu++17 -fsyntax-only -Wall -DEXO_ACQ_DIAG_HOST_TEST=1 $includes $path 2>&1
    if ($LASTEXITCODE -eq 0) {
        Write-Output "PASS $relative"
    } else {
        $failed++
        Write-Output "FAIL $relative"
        $output | ForEach-Object { Write-Output "     $_" }
    }
}

if ($failed -ne 0) {
    Write-Error "$failed host suite(s) failed to compile."
    exit 1
}

Write-Output "PASS: host syntax suite ($($sources.Count) sources)"

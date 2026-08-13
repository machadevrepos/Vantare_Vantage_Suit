# Compile-checks the host test suites.
#
# There is no native host compiler in this tree, so the behavioural suites are
# written as constexpr/static_assert and evaluated by `arm-none-eabi-g++
# -fsyntax-only`. A regression is a compile error, not a silent pass. See the
# header comment in Firmware/Master/tests/acquisition_diagnostics_test.cpp.
#
#   pwsh -File Firmware/HostTests/run_host_syntax_suite.ps1

$ErrorActionPreference = 'Stop'
$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')

$compiler = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-g++.exe'
if (-not (Test-Path $compiler)) {
    Write-Warning "ARM compiler not found; skipping host syntax suite."
    exit 0
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
    'Firmware\Master\tests\master_training_csv_coordinator_test.cpp',
    'Firmware\Master\tests\master_node_session_stager_test.cpp',
    'Firmware\Master\tests\master_session_timestamp_ledger_test.cpp',
    'Firmware\Master\tests\acquisition_diagnostics_test.cpp'
)

$failed = 0
foreach ($relative in $sources) {
    $path = Join-Path $repo $relative
    if (-not (Test-Path $path)) {
        Write-Output "SKIP $relative (not found)"
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

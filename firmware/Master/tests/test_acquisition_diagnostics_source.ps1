$ErrorActionPreference = 'Stop'

$diagPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\ACQUISITION_DIAGNOSTICS.h'
$hubPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\HUB_SENSOR_TEST_APP.h'
$bnoPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\BNO85_STM32.h'
$recorderPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\MASTER_SD_SESSION_RECORDER.h'
$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'

$diag = Get-Content -Raw $diagPath
$hub = Get-Content -Raw $hubPath
$bno = Get-Content -Raw $bnoPath
$recorder = Get-Content -Raw $recorderPath
$main = Get-Content -Raw $mainPath

$failures = [System.Collections.Generic.List[string]]::new()

# --- Every experiment switch must default off so production builds are unchanged.
$experiments = @(
    'EXO_ACQ_DIAG_SUPPRESS_SD',
    'EXO_ACQ_DIAG_QUIET_COMMS',
    'EXO_ACQ_DIAG_ICM_ONLY',
    'EXO_ACQ_DIAG_BNO_ONLY',
    'EXO_ACQ_DIAG_BNO_RV_ONLY'
)
foreach ($macro in $experiments) {
    if ($diag -notmatch "#ifndef\s+$macro\s*\r?\n#define\s+$macro\s+0") {
        $failures.Add("Diagnostic experiment $macro must be an explicit macro that defaults to 0.")
    }
}
if ($diag -notmatch '#ifndef\s+EXO_ACQ_DIAG_ENABLE\s*\r?\n#define\s+EXO_ACQ_DIAG_ENABLE\s+1') {
    $failures.Add('Acquisition counters must be compile-time gated by EXO_ACQ_DIAG_ENABLE.')
}
if ($diag -notmatch 'EXO_ACQ_DIAG_QUIET_COMMS\s*&&\s*!EXO_ACQ_DIAG_ENABLE[\s\S]{0,120}#error') {
    $failures.Add('Quiet-comms mode must fail the build when the counters it reads are compiled out.')
}
if ($diag -notmatch 'EXO_ACQ_DIAG_ICM_ONLY\s*&&\s*EXO_ACQ_DIAG_BNO_ONLY[\s\S]{0,120}#error') {
    $failures.Add('ICM-only and BNO-only capture modes must be mutually exclusive at compile time.')
}

# --- Sub-millisecond timing must not come from the 1 ms tick.
if ($diag -notmatch 'DWT->CYCCNT' -or $diag -notmatch 'CoreDebug_DEMCR_TRCENA_Msk') {
    $failures.Add('The diagnostic clock must be the free-running DWT cycle counter.')
}
# The header names HAL_GetTick() in a comment explaining why it is unfit; only
# an actual read of it is a failure.
if ($diag -match '(=|return)\s*HAL_GetTick\(\)') {
    $failures.Add('HAL_GetTick() is 1 ms resolution and must not back sub-millisecond diagnostics.')
}

# --- All four required latency buckets, plus maximum tracking.
foreach ($bucket in @('over_5ms', 'over_10ms', 'over_20ms', 'over_100ms')) {
    if ($diag -notmatch "\+\+$bucket") {
        $failures.Add("LatencyStat must account for the $bucket bucket.")
    }
}
if ($diag -notmatch 'if\s*\(duration_us\s*>\s*max_us\)') {
    $failures.Add('LatencyStat must track the maximum observed duration.')
}
if ($diag -notmatch 'constexpr void reset\(\)' -or $diag -notmatch 'constexpr void begin_session') {
    $failures.Add('Counters must expose a constexpr reset and per-session start so tests can drive them.')
}

# --- Exactly one summary per session, and never inside the capture loop.
if ($diag -notmatch 'constexpr bool claim_summary\(\)[\s\S]*?summary_pending\s*=\s*false;[\s\S]*?\+\+summaries_emitted;') {
    $failures.Add('claim_summary must consume the pending flag so a session summary cannot repeat.')
}
if ($main -notmatch 'if\s*\(g_acq_diag\.claim_summary\(\)\)\s*\{[\s\S]{0,200}?acq_diag_emit_summary\(\);') {
    $failures.Add('The Master superloop must emit the summary only through claim_summary().')
}
if ($main -notmatch 'g_acq_diag\.end_session\(') {
    $failures.Add('Capture end must queue the diagnostic summary.')
}
if ($main -notmatch 'g_acq_diag\.begin_session\(') {
    $failures.Add('Capture start must reset the diagnostic counters.')
}

# --- No per-sample logging or storage work in the acquisition hot path.
if ($hub -match 'EXO_LOG' -or $hub -match 'printf') {
    $failures.Add('HubSensorTestApp::process() must stay free of per-sample logging.')
}
$processBody = [regex]::Match($hub, 'HubSensorSnapshot process\(\)[\s\S]*?\n    \}').Value
if ($processBody -eq '') {
    $failures.Add('Unable to locate HubSensorTestApp::process() for hot-path inspection.')
} elseif ($processBody -match 'f_write|f_sync|snprint') {
    $failures.Add('The acquisition hot path must not perform storage or formatting work.')
}

# --- Per-sensor instrumentation is actually wired up.
foreach ($counter in @('bno_service_calls', 'bno_take_latest_ok', 'icm_attempts', 'icm_ok', 'icm_fail')) {
    if ($hub -notmatch [regex]::Escape($counter)) {
        $failures.Add("HubSensorTestApp must record the $counter counter.")
    }
}
if ($hub -notmatch 'd->bno_gap\.note\(' -or $hub -notmatch 'd->icm_gap\.note\(') {
    $failures.Add('Both sensors must record the time between captured samples.')
}
if ($hub -notmatch '#if\s+!EXO_ACQ_DIAG_ICM_ONLY[\s\S]*?bno85_\.service\(\)') {
    $failures.Add('ICM-only capture mode must compile out the BNO service call.')
}
if ($hub -notmatch '#if\s+!EXO_ACQ_DIAG_BNO_ONLY[\s\S]*?icm45686_\.read_sample\(') {
    $failures.Add('BNO-only capture mode must compile out the ICM read.')
}

# --- BNO report accounting and the rotation-vector-only experiment.
if ($bno -notmatch 'report_counts_\[::exo::diag::bno_slot_for_report\(event->reportId\)\]\+\+') {
    $failures.Add('The BNO driver must count sensor events by report id.')
}
if ($bno -notmatch '#if\s+EXO_ACQ_DIAG_BNO_RV_ONLY[\s\S]*?const int la_status\s*=\s*SH2_OK') {
    $failures.Add('Rotation-vector-only mode must skip the auxiliary BNO report configuration.')
}
if ($bno -notmatch 'sh2_setSensorConfig\(SH2_GAME_ROTATION_VECTOR') {
    $failures.Add('The game rotation vector must always be configured.')
}

# --- Recorder write timing wraps the sample-batch writes only.
if ($recorder -notmatch 'FRESULT timed_write\(' ) {
    $failures.Add('The recorder must route sample-batch writes through a timed wrapper.')
}
if ($recorder -notmatch 'const FRESULT result = timed_write\(bno_buffer_' -or
    $recorder -notmatch 'const FRESULT result = timed_write\(icm_buffer_') {
    $failures.Add('Both recorder flush paths must be timed.')
}
if ($recorder -notmatch 'd->sd_write\.note\(') {
    $failures.Add('Recorder write durations must feed the latency buckets.')
}

# --- Comms instrumentation covers the three major superloop service calls.
foreach ($pair in @(
        @('comms_ble', 'MX_APPE_Process\(\)'),
        @('comms_leaf', 'leaf_ble_manager\.process\(\)'),
        @('comms_central', 'exo_hub_central_client_process\(\)'))) {
    $stat = $pair[0]
    $call = $pair[1]
    if ($main -notmatch "EXO_ACQ_DIAG_SCOPE\(g_acq_diag\.$stat\);\s*\r?\n\s*$call;") {
        $failures.Add("The $stat service call must be timed during capture.")
    }
}

# --- The disabled build must keep the original call sequence.
if ($main -notmatch '#else\s*\r?\n\s*leaf_ble_manager\.process\(\);\s*\r?\n\s*exo_hub_central_client_process\(\);\s*\r?\n#endif') {
    $failures.Add('A diagnostics-disabled build must preserve the untimed service call sequence.')
}
if ($main -notmatch '#else\s*\r?\n\s*MX_APPE_Process\(\);\s*\r?\n#endif') {
    $failures.Add('A diagnostics-disabled build must call MX_APPE_Process() unwrapped.')
}

# --- Suppress-SD keeps acquisition running but skips the recorder append.
if ($main -notmatch '#if\s+EXO_ACQ_DIAG_SUPPRESS_SD[\s\S]*?const bool bno_stored = true;[\s\S]*?#else[\s\S]*?append_bno85') {
    $failures.Add('Suppress-SD mode must bypass the BNO recorder append without disabling acquisition.')
}
if ($main -notmatch '#if\s+EXO_ACQ_DIAG_SUPPRESS_SD[\s\S]*?const bool icm_stored = true;[\s\S]*?#else[\s\S]*?append_icm45686') {
    $failures.Add('Suppress-SD mode must bypass the ICM recorder append without disabling acquisition.')
}

# --- Never sensor I2C, FatFs, BLE or printf from an ISR.
if ($diag -match 'HAL_I2C_|f_write|f_sync|EXO_LOG') {
    $failures.Add('The diagnostics header must not perform I2C, storage or logging work.')
}

# --- The constexpr behavior suite must actually compile (and therefore run).
$compiler = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-g++.exe'
if (Test-Path $compiler) {
    $testCpp = Join-Path $PSScriptRoot 'acquisition_diagnostics_test.cpp'
    $includeDir = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM'
    $output = & $compiler -std=gnu++17 -fsyntax-only -Wall -I $includeDir $testCpp 2>&1
    if ($LASTEXITCODE -ne 0) {
        $failures.Add("acquisition_diagnostics_test.cpp static_assert suite failed: $output")
    }
} else {
    Write-Warning "ARM compiler not found; skipped the constexpr behavior suite."
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'PASS: acquisition diagnostics instrumentation and experiment-switch contract'

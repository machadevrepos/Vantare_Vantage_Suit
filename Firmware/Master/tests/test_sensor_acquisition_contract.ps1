$ErrorActionPreference = 'Stop'

$icmPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\ICM45686_STM32.h'
$hubPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\HUB_SENSOR_TEST_APP.h'
$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$icm = Get-Content -Raw $icmPath
$hub = Get-Content -Raw $hubPath
$main = Get-Content -Raw $mainPath
$failures = [System.Collections.Generic.List[string]]::new()

if ($icm -notmatch 'sample\.sequence\s*=\s*next_sequence_\+\+') {
    $failures.Add('Successful ICM V4 acquisitions must receive a monotonic sensor sequence.')
}
if ($icm -notmatch 'uint32_t\s+next_sequence_\s*=\s*0U') {
    $failures.Add('The ICM driver must own a zero-initialized V4 sequence counter.')
}
if ($main -match 'icm_sample\.sequence\s*=\s*g_local_session_recorder\.header\(\)\.icm45686_sample_count') {
    $failures.Add('Master recording must not reuse the committed ICM count as every buffered sample sequence.')
}
$recorderPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\MASTER_SD_SESSION_RECORDER.h'
$recorder = Get-Content -Raw $recorderPath
if ($recorder -notmatch 'Icm45686Sample\s+sequenced\s*=\s*sample;[\s\S]*?sequenced\.sequence\s*=\s*header_\.icm45686_sample_count\s*\+\s*icm_buffer_count_') {
    $failures.Add('Master recorder must assign each accepted buffered ICM sample from committed plus buffered count.')
}
if ($recorder -notmatch '!recording_\s*\|\|\s*write_failed_\s*\|\|\s*bno_buffer_count_\s*>=\s*kBnoBufferSamples') {
    $failures.Add('Master recorder must latch failed writes and bounds-check BNO before indexing its batch buffer.')
}
if ($recorder -notmatch '!recording_\s*\|\|\s*write_failed_\s*\|\|\s*icm_buffer_count_\s*>=\s*kIcmBufferSamples') {
    $failures.Add('Master recorder must latch failed writes and bounds-check ICM before indexing its batch buffer.')
}
if ($hub -notmatch '#ifndef\s+EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE[\s\S]*?#define\s+EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE\s+0') {
    $failures.Add('The blocking HUBTEST.BIN background logger must be disabled by default.')
}
if ($hub -notmatch '#if\s+EXO_HUB_SENSOR_BACKGROUND_LOG_ENABLE[\s\S]*?append_bin_record\(snapshot\)[\s\S]*?#endif') {
    $failures.Add('HUBTEST.BIN writes must be compile-time gated outside the acquisition hot path.')
}
if ($main -notmatch '#ifndef\s+EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE[\s\S]*?#define\s+EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE\s+0') {
    $failures.Add('The continuous master IMU CSV logger must be disabled by default.')
}
$csvGates = [regex]::Matches(
    $main,
    '#if\s+EXO_MASTER_IMU_BACKGROUND_CSV_ENABLE[\s\S]*?#endif'
)
if ($csvGates.Count -lt 2 -or
    -not ($csvGates.Value -match 'master_imu_csv_logger\.begin') -or
    -not ($csvGates.Value -match 'master_imu_csv_logger\.append_bno') -or
    -not ($csvGates.Value -match 'master_imu_csv_logger\.shutdown')) {
    $failures.Add('Master IMU CSV startup, hot-loop writes, and shutdown must be compile-time gated.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'PASS: sensor acquisition metadata and non-blocking background logging contract'

$ErrorActionPreference = 'Stop'

$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$mainSource = Get-Content -Raw $mainPath
$hubAppPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\HUB_SENSOR_TEST_APP.h'
$hubAppSource = Get-Content -Raw $hubAppPath
$formatterPath = Join-Path $PSScriptRoot '..\Core\Inc\MASTER_IMU_CSV_FORMATTER.h'
$formatterSource = Get-Content -Raw $formatterPath
$fatFsConfigPath = Join-Path $PSScriptRoot '..\FATFS\Target\ffconf.h'
$fatFsConfigSource = Get-Content -Raw $fatFsConfigPath
$sessionRecorderPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\MASTER_SD_SESSION_RECORDER.h'
$sessionRecorderSource = Get-Content -Raw $sessionRecorderPath
$failures = [System.Collections.Generic.List[string]]::new()

$requiredMainPatterns = @(
    '#include <MASTER_IMU_CSV_LOGGER.h>',
    'static exo::MasterImuCsvLogger master_imu_csv_logger',
    'master_imu_csv_logger.begin(',
    'master_imu_csv_logger.append_bno(',
    'master_imu_csv_logger.append_icm(',
    'master_imu_csv_logger.service(csv_now_ms)',
    'hub_sensor_test_app.bno_available_mask()',
    'master_imu_csv_logger.shutdown()',
    '[IMU][CSV] opened',
    '[IMU][CSV] disabled',
    '[IMU][CSV] shutdown'
)

foreach ($pattern in $requiredMainPatterns) {
    if (-not $mainSource.Contains($pattern)) {
        $failures.Add("Missing Master CSV integration contract: $pattern")
    }
}

if (-not $mainSource.Contains('static_cast<uint64_t>(csv_now_ms) * 1000ULL')) {
    $failures.Add('CSV integration must create a 64-bit microsecond timestamp before multiplication.')
}

if (-not $hubAppSource.Contains('append_bin_record(snapshot);')) {
    $failures.Add('Existing HUBTEST.BIN append call was removed or changed.')
}

if (-not $hubAppSource.Contains('/SESSIONS/HUBTEST.BIN')) {
    $failures.Add('Existing HUBTEST.BIN path was removed or changed.')
}

$header = ($formatterSource -split "`r?`n" | Where-Object { $_ -like '*schema_version,row_sequence,sensor_sequence*' } | Select-Object -First 1).Trim()
if (-not $header) {
    $failures.Add('CSV header declaration was not found.')
} else {
    $header = $header.Trim('"').TrimEnd('\','n','r')
}

if ($formatterSource -notmatch 'static constexpr char kCsvHeader\[\]') {
    $failures.Add('CSV formatter must expose the fixed header.')
}

if ($fatFsConfigSource -notmatch '#define\s+_FS_LOCK\s+(?:[3-9]|[1-9][0-9]+)\b') {
    $failures.Add('Master FatFs must retain at least three locks for HUBTEST.BIN, boot CSV, and the session recorder.')
}

if ($sessionRecorderSource -notmatch 'if\s*\(\s*::USERFatFs\.fs_type\s*==\s*0U\s*\)\s*\{[\s\S]*?f_mount\(&::USERFatFs,\s*::USERPath,\s*1U\)') {
    $failures.Add('Master session start must not remount and invalidate the already-open HUBTEST.BIN and CSV files.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'Master IMU CSV source contract passed.'

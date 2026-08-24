$ErrorActionPreference = 'Stop'

$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$source = Get-Content -Raw $mainPath
$bnoDriverPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\BNO85_STM32.h'
$bnoDriverSource = Get-Content -Raw $bnoDriverPath
$hubAppPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\HUB_SENSOR_TEST_APP.h'
$hubAppSource = Get-Content -Raw $hubAppPath
$failures = [System.Collections.Generic.List[string]]::new()

$requiredPatterns = @(
    '#include <MASTER_IMU_SWO_TELEMETRY.h>',
    '#define EXO_MASTER_IMU_SWO_ENABLE 1',
    '#define EXO_MASTER_IMU_SWO_INTERVAL_MS exo::imu_swo::kDefaultIntervalMs',
    'last_icm_sample',
    'have_last_icm',
    '[IMU][BNO][Q]',
    '[IMU][BNO][MOTION]',
    '[IMU][ICM][RAW]',
    '[IMU][ICM][SCALED]',
    'scale_quaternion_1e4',
    'scale_bno_milli',
    'scale_icm_accel_mg',
    'scale_icm_gyro_mdps',
    'hub_sensor_test_app.bno_available_mask()',
    '(bno_available_mask & 0x0EU) == 0x0EU'
)

foreach ($pattern in $requiredPatterns) {
    if (-not $source.Contains($pattern)) {
        $failures.Add("Missing required source contract: $pattern")
    }
}

if ($source -notmatch '#ifndef EXO_MASTER_IMU_SWO_ENABLE[\s\S]*?#define EXO_MASTER_IMU_SWO_ENABLE 1') {
    $failures.Add('The telemetry enable switch is not overridable with #ifndef.')
}

if ($source -notmatch '#ifndef EXO_MASTER_IMU_SWO_INTERVAL_MS[\s\S]*?#define EXO_MASTER_IMU_SWO_INTERVAL_MS exo::imu_swo::kDefaultIntervalMs') {
    $failures.Add('The telemetry interval switch is not overridable with #ifndef.')
}

if ($source -notmatch '#define EXO_MASTER_VERBOSE_DIAG 0') {
    $failures.Add('EXO_MASTER_VERBOSE_DIAG must remain disabled by default.')
}

if (-not $bnoDriverSource.Contains('uint8_t available_mask() const { return latest_available_mask_; }')) {
    $failures.Add('BNO85 driver does not expose its accumulated subreport availability mask.')
}

if (-not $hubAppSource.Contains('uint8_t bno_available_mask() const { return bno85_.available_mask(); }')) {
    $failures.Add('Hub sensor app does not expose the BNO85 subreport availability mask.')
}

$expectedFormats = @(
    '[IMU][BNO][Q] t_ms=%lu av=0x%02X q1e4=%ld,%ld,%ld,%ld`r`n',
    '[IMU][BNO][MOTION] lin_mms2=%ld,%ld,%ld grav_mms2=%ld,%ld,%ld gyro_mrads=%ld,%ld,%ld`r`n',
    '[IMU][ICM][RAW] t_ms=%lu accel_lsb=%d,%d,%d gyro_lsb=%d,%d,%d`r`n',
    '[IMU][ICM][SCALED] accel_mg=%ld,%ld,%ld gyro_mdps=%ld,%ld,%ld`r`n'
)

foreach ($format in $expectedFormats) {
    $sourceFormat = $format.Replace('`r', '\r').Replace('`n', '\n')
    if (-not $source.Contains($sourceFormat)) {
        $failures.Add("Missing exact bounded format string: $sourceFormat")
    }
    if ($format.Length -ge 160) {
        $failures.Add("Format string exceeds SWO printer limit: $format")
    }
}

$worstCaseLines = @(
    "[IMU][BNO][Q] t_ms=4294967295 av=0x0F q1e4=-10000,-10000,-10000,-10000`r`n",
    "[IMU][BNO][MOTION] lin_mms2=-2147483648,-2147483648,-2147483648 grav_mms2=-2147483648,-2147483648,-2147483648 gyro_mrads=-2147483648,-2147483648,-2147483648`r`n",
    "[IMU][ICM][RAW] t_ms=4294967295 accel_lsb=-32768,-32768,-32768 gyro_lsb=-32768,-32768,-32768`r`n",
    "[IMU][ICM][SCALED] accel_mg=-4000,-4000,-4000 gyro_mdps=-2000000,-2000000,-2000000`r`n"
)

foreach ($line in $worstCaseLines) {
    if ($line.Length -ge 160) {
        $failures.Add("Worst-case expanded line exceeds SWO printer limit ($($line.Length) bytes): $line")
    }
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'Master IMU SWO source contract passed.'

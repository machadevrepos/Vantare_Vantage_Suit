$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$types = Get-Content -Raw (Join-Path $root 'LIBRARY/CUSTOM/RECORDING_TYPES.h')
$app = Get-Content -Raw (Join-Path $root 'LIBRARY/CUSTOM/NODE_RECORDING_APP.h')
$main = Get-Content -Raw (Join-Path $root 'Node/Core/Src/main.c')

function Require-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Require-NoMatch([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -match $Pattern) {
        throw $Message
    }
}

Require-Match $types 'EXO_SAMPLE_FORMAT_VERSION\s+4U' `
    'The lossless session format must default to version 4.'
Require-Match $types 'bno85_target_rate_hz' `
    'The session header must declare the BNO target rate.'
Require-Match $types 'icm45686_target_rate_hz' `
    'The session header must declare the ICM target rate.'
Require-Match $types 'bno85_attempted_count' `
    'The session header must track BNO attempted samples.'
Require-Match $types 'icm45686_attempted_count' `
    'The session header must track ICM attempted samples.'
Require-Match $types 'bno85_dropped_count' `
    'The session header must track BNO drops.'
Require-Match $types 'icm45686_dropped_count' `
    'The session header must track ICM drops.'
Require-Match $types 'struct Icm45686SampleV4[\s\S]*offset_us[\s\S]*sequence' `
    'ICM v4 samples must carry capture time and sequence.'

Require-Match $app 'kBnoTargetRateHz\s*=\s*400U' `
    'Node capture must record BNO at the sensor-native 400 Hz queue rate.'
Require-Match $app 'kIcmPeriodUs\s*=\s*5000U' `
    'Node capture must schedule ICM at 200 Hz.'
Require-Match $app 'bno85_\.pop_samples\(' `
    'Node capture must drain BNO samples from the sensor-driven capture queue.'
Require-Match $app 'set_capture_queue_enabled\(true\)' `
    'Node capture must enable the BNO capture queue at session start.'
Require-Match $app 'record_next_icm_us_' `
    'Node capture must maintain an independent ICM deadline.'
Require-NoMatch $app 'kRecordTickMs\s*=\s*10U' `
    'The shared 10 ms recording tick must be removed.'

Require-NoMatch $main '#include\s+<RS485_RECORD_NODE_APP\.h>' `
    'The BLE-only Node must not include the RS-485 responder.'
Require-Match $main 'class\s+BleOnlyNodeResponder' `
    'The Node must keep stream control state without activating UART transport.'
Require-Match $main 'node_recording_app\.process\(\);[\s\S]*node_blepipe_process_recording_upload\(\);' `
    'The BLE-only Node loop must service recording and BLE upload.'
Require-NoMatch $main 'node_rs485_recording\.process\(\);' `
    'The BLE-only Node loop must not service the UART responder.'

Write-Output 'PASS: lossless Node recording source contract'

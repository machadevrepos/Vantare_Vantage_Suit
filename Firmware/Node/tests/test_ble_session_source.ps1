$ErrorActionPreference = 'Stop'

$nodeRoot = Split-Path -Parent $PSScriptRoot
$main = Get-Content -Raw (Join-Path $nodeRoot 'Core/Src/main.c')
$recording = Get-Content -Raw (Join-Path $nodeRoot '..\LIBRARY\CUSTOM\NODE_RECORDING_APP.h')

function Require-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) { throw $Message }
}

Require-Match $main 'BLEPIPE_MSG_LEAF_SAMPLE' `
    'Node live samples must use the BLEPipe leaf-sample message.'
Require-Match $main 'node_recording_app\.peek_live_sample' `
    'The Node main loop must retain a live sample until BLE accepts it.'
Require-Match $main 'node_recording_app\.discard_live_sample' `
    'The Node main loop must discard a live sample only after successful BLE submission.'
Require-Match $main 'RecordCommand::StopRecord' `
    'The Node BLE command dispatcher must accept StopRecord.'
Require-Match $recording 'pending_start_\.session_id\s*!=\s*message\.session_id' `
    'StopRecord must reject a different session ID.'
Require-Match $recording 'recorder_\.finalize\(finalize_duration_ms_\)' `
    'Node finalization must use measured elapsed time.'

$bnoRecord = $recording.IndexOf('bno_record_buf_.enqueue(sample)')
$bnoLive = $recording.IndexOf('live_queue_.offer(kBnoLiveSensorId')
$icmRecord = $recording.IndexOf('icm_record_buf_.enqueue(sample)')
$icmLive = $recording.IndexOf('live_queue_.offer(kIcmLiveSensorId')
if ($bnoRecord -lt 0 -or $bnoLive -le $bnoRecord -or
    $icmRecord -lt 0 -or $icmLive -le $icmRecord) {
    throw 'Each sensor sample must enter the loss-resistant record buffer before its best-effort live queue.'
}

Write-Output 'PASS: Node BLE coordinated-session source contract'

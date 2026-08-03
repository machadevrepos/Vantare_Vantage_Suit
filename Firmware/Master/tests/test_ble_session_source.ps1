$ErrorActionPreference = 'Stop'

$masterRoot = Split-Path -Parent $PSScriptRoot
$main = Get-Content -Raw (Join-Path $masterRoot 'Core/Src/main.c')

function Require-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) { throw $Message }
}

function Require-NoMatch([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -match $Pattern) { throw $Message }
}

Require-Match $main 'RecordCommand::StartSession' `
    'Master must accept the atomic StartSession command.'
Require-Match $main 'session_missing_node_mask\(requested_node_mask,\s*ready_node_mask\)' `
    'Master must reject selected Nodes that are not recorder-ready.'
Require-Match $main 'g_active_session_node_mask\s*=\s*g_record_sync\.target_mask' `
    'Master must retain the immutable committed participant mask.'
Require-Match $main 'session_expected_source_mask\(g_record_sync\.target_mask\)' `
    'CSV expected sources must be derived from the committed participant mask.'
Require-Match $main 'RecordCommand::StopRecord' `
    'Master must accept and forward session-scoped StopRecord.'
Require-Match $main 'record_sync_send_to_target_mask\(g_active_session_node_mask' `
    'Stop and live control must target only the committed participants.'
Require-Match $main 'g_local_finalize_duration_ms\s*=\s*elapsed_ms' `
    'Master finalization must use measured elapsed time.'
Require-Match $main 'master_rs485_recording\.peek_next_live_sample' `
    'Master must retain a Node live sample while the phone BLE link is busy.'
Require-Match $main 'master_rs485_recording\.discard_next_live_sample' `
    'Master must remove a Node live sample only after successful phone BLE submission.'
Require-NoMatch $main 'standard dataset requires NODE1\.\.NODE4' `
    'Master must not require a fixed four-Node dataset.'

Write-Output 'PASS: Master BLE coordinated-session source contract'

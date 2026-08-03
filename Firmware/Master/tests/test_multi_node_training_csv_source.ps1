$ErrorActionPreference = 'Stop'

$bridgePath = Join-Path $PSScriptRoot '..\Core\Inc\EXO_HUB_LEAF_BRIDGE.h'
$bridgeSource = Get-Content -Raw $bridgePath
$centralPath = Join-Path $PSScriptRoot '..\STM32_WPAN\App\exo_hub_central_client.c'
$centralSource = Get-Content -Raw $centralPath
$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$mainSource = Get-Content -Raw $mainPath
$ffconfPath = Join-Path $PSScriptRoot '..\FATFS\Target\ffconf.h'
$ffconfSource = Get-Content -Raw $ffconfPath
$hubAppPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\HUB_SENSOR_TEST_APP.h'
$hubAppSource = Get-Content -Raw $hubAppPath
$sessionRecorderPath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\MASTER_SD_SESSION_RECORDER.h'
$sessionRecorderSource = Get-Content -Raw $sessionRecorderPath
$coordinatorPath = Join-Path $PSScriptRoot '..\Core\Inc\MASTER_TRAINING_CSV_COORDINATOR.h'
$coordinatorSource = Get-Content -Raw $coordinatorPath
$stagerPath = Join-Path $PSScriptRoot '..\Core\Inc\MASTER_NODE_SESSION_STAGER.h'
$stagerSource = Get-Content -Raw $stagerPath
$failures = [System.Collections.Generic.List[string]]::new()

if ($bridgeSource -notmatch 'void\s+exo_hub_leaf_record_frame_ingest\s*\(\s*uint8_t\s+node_id,\s*const\s+uint8_t\s*\*payload,\s*uint16_t\s+payload_len\s*\);') {
    $failures.Add('Missing exact void reliable-frame bridge declaration contract.')
}

$requiredMainPatterns = @(
    '#include <MASTER_TRAINING_CSV_COORDINATOR.h>',
    'static exo::MasterTrainingCsvCoordinator master_training_csv_coordinator',
    'extern "C" void exo_hub_leaf_record_frame_ingest',
    'master_training_csv_coordinator.on_node_record_done(message)',
    'master_training_csv_coordinator.on_node_reliable_frame(node_id, payload, payload_len)'
)
foreach ($pattern in $requiredMainPatterns) {
    if (-not $mainSource.Contains($pattern)) {
        $failures.Add("Missing training coordinator bridge contract: $pattern")
    }
}

$resetStart = $mainSource.IndexOf('static uint8_t master_record_reset_all(bool erase_remote)')
$resetEnd = $mainSource.IndexOf('static void ble_send_discovered_nodes_report()', $resetStart)
if ($resetStart -lt 0 -or $resetEnd -le $resetStart) {
    throw 'Unable to isolate master_record_reset_all for training cleanup contract.'
}
$resetAll = $mainSource.Substring($resetStart, $resetEnd - $resetStart)
if (-not $resetAll.Contains('master_training_csv_coordinator.shutdown(HAL_GetTick())')) {
    throw 'Master reset-all must attempt training CSV/stager cleanup before starting another session.'
}

$requiredLifecyclePatterns = @(
    'master_training_csv_coordinator.begin_session(message.session_id, expected_mask)',
    'master_training_csv_coordinator.note_master_icm_time(',
    'master_training_csv_coordinator.on_master_finalized(g_local_session_recorder)',
    'master_training_csv_coordinator.service(g_local_session_recorder, HAL_GetTick())',
    'master_training_csv_coordinator.shutdown(HAL_GetTick())',
	'master_training_csv_coordinator.cancel_session()',
	'static bool local_record_start',
	'if (!local_record_start(g_record_sync.message, true))',
	'master_training_csv_replay_pending_node_done()',
	'master_training_csv_node_manifest_ready(g_pending_node_done)',
	'g_training_node_done[4]',
	'g_training_node_done_valid[4]',
    '[TRAIN][CSV] opened',
    '[TRAIN][CSV] source complete',
    '[TRAIN][CSV] incomplete'
)
foreach ($pattern in $requiredLifecyclePatterns) {
    if (-not $mainSource.Contains($pattern)) {
        $failures.Add("Missing training CSV lifecycle contract: $pattern")
    }
}

if ($ffconfSource -notmatch '(?m)^\s*#define\s+_FS_LOCK\s+5(?:\s|$)') {
    $failures.Add('FatFs _FS_LOCK must be 5 for HUBTEST.BIN, IMUxxxx.CSV, MREC.BIN, TRNxxxx.CSV, and NTMP.BIN.')
}

$incrementalValidationContracts = @(
    'kValidationBytesPerService = 256U',
    'stager_.begin_validation()',
    'stager_.step_validation(kValidationBytesPerService)',
    'stager_.finalize_validation()'
)
foreach ($pattern in $incrementalValidationContracts) {
    if (-not $coordinatorSource.Contains($pattern)) {
        $failures.Add("Missing bounded coordinator validation contract: $pattern")
    }
}
if ($coordinatorSource.Contains('finish_and_validate()') -or
        $stagerSource.Contains('bool finish_and_validate()')) {
    $failures.Add('Node validation must not retain the synchronous full-file finish_and_validate path.')
}

if (-not $coordinatorSource.Contains('append_bno(0U, sample.offset_us, sample, false, 0U, now_ms)')) {
    $failures.Add('Master BNO training rows must leave availability blank; SessionHeader.sensor_mask is sensor presence, not BNO field availability.')
}

$verifyStart = $mainSource.IndexOf('case exo::RecordReliableType::VerifyOk:')
$verifyEnd = $mainSource.IndexOf('case exo::RecordReliableType::VerifyFail:', $verifyStart + 1)
if ($verifyStart -lt 0 -or $verifyEnd -le $verifyStart) {
    $failures.Add('VerifyOk handler block was not found.')
} else {
    $verifyBlock = $mainSource.Substring($verifyStart, $verifyEnd - $verifyStart)
    $phoneAck = $verifyBlock.IndexOf('Custom_APP_SendCmdAck(payload, length, 1U)')
    $holdDecision = $verifyBlock.IndexOf('master_training_csv_should_hold_node_verify(verify)')
    if ($phoneAck -lt 0 -or $holdDecision -lt 0 -or $phoneAck -ge $holdDecision) {
        $failures.Add('Phone VerifyOk ACK must remain immediate before the training CSV hold decision.')
    }
    if (-not $verifyBlock.Contains('training_pending_verify_store(verify, payload, length)')) {
        $failures.Add('Matching Node VerifyOk must be retained in bounded per-Node storage while CSV completion is pending.')
    }
}

$verifyReleaseContracts = @(
    'static TrainingPendingVerifyOk g_training_pending_verify_ok[4]',
    'master_training_csv_coordinator.completed_source_mask()',
    'master_rs485_recording.on_ble_reliable_verify_ok(',
    'forward_remote_record_control(verify.source_id, payload, length)',
    'start_next_pending_node_manifest_now()',
    'memset(g_training_pending_verify_ok, 0, sizeof(g_training_pending_verify_ok))'
)
foreach ($pattern in $verifyReleaseContracts) {
    if (-not $mainSource.Contains($pattern)) {
        $failures.Add("Missing deferred Node VerifyOk release contract: $pattern")
    }
}
if (-not $mainSource.Contains('state == exo::TrainingCsvState::Idle || state == exo::TrainingCsvState::Complete')) {
    $failures.Add('Idle/complete coordinators must not hold late or unrelated Node VerifyOk frames.')
}
if (-not $mainSource.Contains('state == exo::TrainingCsvState::CsvError ||') -or
        -not $mainSource.Contains('state == exo::TrainingCsvState::StageError')) {
    $failures.Add('CSV/staging terminal errors must retain held VerifyOk approvals even after a source completion bit was set.')
}

$preservedContracts = @(
    'master_imu_csv_logger.service(csv_now_ms)',
    'master_imu_csv_logger.shutdown()',
    'HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET)'
)
foreach ($pattern in $preservedContracts) {
    if (-not $mainSource.Contains($pattern)) {
        $failures.Add("Existing recording/power contract was removed or changed: $pattern")
    }
}
if (-not $hubAppSource.Contains('append_bin_record(snapshot);') -or
        -not $hubAppSource.Contains('/SESSIONS/HUBTEST.BIN')) {
    $failures.Add('Existing HUBTEST.BIN capture path was removed or changed.')
}
if (-not $sessionRecorderSource.Contains('/SESSIONS/MREC.BIN')) {
    $failures.Add('Existing MREC.BIN session path was removed or changed.')
}

$recordDoneStart = $mainSource.IndexOf('extern "C" uint8_t exo_hub_leaf_record_done_ingest')
$recordDoneEnd = $mainSource.IndexOf('extern "C" void exo_hub_leaf_record_frame_ingest', $recordDoneStart + 1)
if ($recordDoneStart -lt 0 -or $recordDoneEnd -le $recordDoneStart) {
    $failures.Add('Node RecordDone ingress function was not found.')
} else {
    $recordDoneIngress = $mainSource.Substring($recordDoneStart, $recordDoneEnd - $recordDoneStart)
    $durableIndex = $recordDoneIngress.IndexOf('g_training_node_done[training_index] = message')
    $observerIndex = $recordDoneIngress.IndexOf('master_training_csv_coordinator.on_node_record_done(message)')
    $transportIndex = $recordDoneIngress.IndexOf('master_rs485_recording.queue_record_done(message)')
    if ($durableIndex -lt 0 -or $observerIndex -lt 0 -or $transportIndex -lt 0 -or
            $durableIndex -ge $observerIndex -or $observerIndex -ge $transportIndex) {
        $failures.Add('RecordDone metadata must become durable before coordinator observation and existing transport queueing.')
    }
}

$manifestStart = $mainSource.IndexOf('if (g_have_pending_node_done &&')
$manifestEnd = $mainSource.IndexOf('if ((g_local_record_phase == LocalRecordPhase::Manifest', $manifestStart + 1)
if ($manifestStart -lt 0 -or $manifestEnd -le $manifestStart) {
    $failures.Add('Pending Node manifest scheduler block was not found.')
} else {
    $manifestBlock = $mainSource.Substring($manifestStart, $manifestEnd - $manifestStart)
    $readyIndex = $manifestBlock.IndexOf('master_training_csv_node_manifest_ready(g_pending_node_done)')
    $sendIndex = $manifestBlock.IndexOf('send_reliable_manifest(')
    if ($readyIndex -lt 0 -or $sendIndex -lt 0 -or $readyIndex -ge $sendIndex) {
        $failures.Add('Node manifest transfer must wait for durable metadata replay into the coordinator.')
    }
}

$poweroffStart = $mainSource.IndexOf('static void poweroff_pcb_and_wait_for_release()')
$poweroffEnd = $mainSource.IndexOf('static void send_touch_status', $poweroffStart + 1)
if ($poweroffStart -lt 0 -or $poweroffEnd -le $poweroffStart) {
    $failures.Add('Controlled power-off function was not found.')
} else {
    $poweroff = $mainSource.Substring($poweroffStart, $poweroffEnd - $poweroffStart)
    $trainingShutdown = $poweroff.IndexOf('master_training_csv_coordinator.shutdown(HAL_GetTick())')
    $bootCsvShutdown = $poweroff.IndexOf('master_imu_csv_logger.shutdown()')
    $powerClear = $poweroff.IndexOf('HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET)')
    if ($trainingShutdown -lt 0 -or $bootCsvShutdown -lt 0 -or $powerClear -lt 0 -or
            $trainingShutdown -ge $bootCsvShutdown -or $bootCsvShutdown -ge $powerClear) {
        $failures.Add('Controlled shutdown must close training CSV, then boot CSV, before clearing PWR_EN.')
    }
}

$rawBranchStart = $centralSource.IndexOf('if (hdr.msg_type == BLEPIPE_MSG_RAW_FORWARD')
$rawBranchEnd = $centralSource.IndexOf('void exo_hub_central_client_init', $rawBranchStart + 1)
if ($rawBranchStart -lt 0 -or $rawBranchEnd -le $rawBranchStart) {
    $failures.Add('BLEPIPE_MSG_RAW_FORWARD branch was not found.')
} else {
    $rawBranch = $centralSource.Substring($rawBranchStart, $rawBranchEnd - $rawBranchStart)
    $observer = 'exo_hub_leaf_record_frame_ingest(exo_leaf_slot_node_id(slot), payload, payload_len)'
    $forward = 'Custom_APP_SendRecordFrame(payload, (uint8_t)payload_len)'
    $observerIndex = $rawBranch.IndexOf($observer)
    $forwardIndex = $rawBranch.IndexOf($forward)
    if ($observerIndex -lt 0) {
        $failures.Add('RAW_FORWARD does not observe the exact decoded payload with the slot Node ID.')
    }
    if ($forwardIndex -lt 0) {
        $failures.Add('Existing byte-for-byte RAW_FORWARD call was removed or changed.')
    }
    if ($observerIndex -ge 0 -and $forwardIndex -ge 0 -and $observerIndex -ge $forwardIndex) {
        $failures.Add('Reliable-frame observation must occur before the existing forwarding call.')
    }
    if ($rawBranch -match 'if\s*\([^\)]*exo_hub_leaf_record_frame_ingest') {
        $failures.Add('RAW_FORWARD must not depend on an observer return value.')
    }
}

if (-not $centralSource.Contains('exo_hub_leaf_record_done_ingest(payload, length')) {
    $failures.Add('Existing RecordDone ingestion path was removed or changed.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'Multi-node training CSV BLE observation source contract passed.'

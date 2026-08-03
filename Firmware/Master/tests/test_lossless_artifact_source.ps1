$ErrorActionPreference = 'Stop'

$masterRoot = Split-Path -Parent $PSScriptRoot
$logger = Get-Content -Raw (Join-Path $masterRoot 'Core/Inc/MASTER_TRAINING_CSV_LOGGER.h')
$stager = Get-Content -Raw (Join-Path $masterRoot 'Core/Inc/MASTER_NODE_SESSION_STAGER.h')
$coordinator = Get-Content -Raw (Join-Path $masterRoot 'Core/Inc/MASTER_TRAINING_CSV_COORDINATOR.h')
$main = Get-Content -Raw (Join-Path $masterRoot 'Core/Src/main.c')

function Require-Match([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -notmatch $Pattern) { throw $Message }
}

function Require-NoMatch([string]$Text, [string]$Pattern, [string]$Message) {
    if ($Text -match $Pattern) { throw $Message }
}

Require-Match $logger '\.TMP' 'Training output must be written to a TMP file first.'
Require-Match $logger 'rename_fn' 'Training completion must rename TMP to CSV.'
Require-Match $logger '\.OK' 'Training completion must create an OK marker last.'
Require-Match $logger 'completed_source_mask_\s*!=\s*expected_source_mask_' `
    'The logger must refuse publication unless every expected source completed.'

Require-Match $stager 'staging_path_\[cursor\+\+\]\s*=\s*''N''' `
    'Node staging must use durable per-session/per-node binary names.'
Require-NoMatch $stager 'NTMP\.BIN' `
    'The validated Node binary must no longer use a disposable NTMP path.'
Require-NoMatch $stager 'unlink_fn\(staging_path\(\)\)' `
    'Validated Node binaries must not be deleted after CSV conversion.'

Require-Match $stager 'loss_flags' `
    'The staging validator must document capture loss as quality metadata.'
Require-NoMatch $stager 'bno85_dropped_count\s*!=\s*0U' `
    'BNO capture loss metadata must not invalidate a structurally sound archive.'
Require-NoMatch $stager 'icm45686_dropped_count\s*!=\s*0U' `
    'ICM capture loss metadata must not invalidate a structurally sound archive.'
Require-NoMatch $stager 'sample_count\s*!=\s*header_\..*captured_count' `
    'Captured-count quality metadata must not invalidate structurally accounted payload samples.'

Require-Match $logger 'if\s*\(\s*!success\s*\)\s*\{\s*return false;\s*\}[\s\S]*?publish\(\)' `
    'A flush, sync, or close failure must stop before CSV/OK publication.'
Require-Match $logger 'publish\(\)[\s\S]*?completed_source_mask_\s*!=\s*expected_source_mask_' `
    'The publish operation itself must enforce complete source coverage.'
Require-Match $main 'session_expected_source_mask\(g_record_sync\.target_mask\)' `
    'The training workflow must derive its expected sources from the immutable selected Node mask.'
Require-NoMatch $main 'standard dataset requires NODE1\.\.NODE4' `
    'A selected one-Node development session must not be blocked by a fixed four-Node requirement.'

Write-Output 'PASS: lossless Master artifact source contract'

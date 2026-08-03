$ErrorActionPreference = 'Stop'

$mainPath = Join-Path $PSScriptRoot '..\Core\Src\main.c'
$source = Get-Content -Raw $mainPath
$failures = [System.Collections.Generic.List[string]]::new()

if (-not $source.Contains('#include <LIVE_STREAM_TIMING.h>')) {
    $failures.Add('Master must include the tested live-stream timing gate.')
}
if ($source -notmatch 'exo::LiveStreamGate\s+bno_stream_gate') {
    $failures.Add('Master must maintain an independent BNO stream gate.')
}
if ($source -notmatch 'exo::LiveStreamGate\s+icm_stream_gate') {
    $failures.Add('Master must maintain an independent ICM stream gate.')
}
if ($source -notmatch 'bno_stream_gate\.accept\(\s*hub_snapshot\.has_bno85') {
    $failures.Add('BNO transmission must be driven by a fresh BNO snapshot.')
}
if ($source -notmatch 'icm_stream_gate\.accept\(\s*hub_snapshot\.has_icm45686') {
    $failures.Add('ICM transmission must be driven by a fresh ICM snapshot.')
}
if ($source -match 'if\s*\(\s*have_last_bno\s*\)[\s\S]{0,1800}?send_ble_v2_sample') {
    $failures.Add('Live BLE must not retransmit a cached BNO sample.')
}
if ($source -notmatch '#(?:if|elif)\s+EXO_SAMPLE_FORMAT_VERSION\s*==\s*4U[\s\S]*?icm_payload\.offset_us\s*=\s*hub_snapshot\.icm45686\.offset_us[\s\S]*?icm_payload\.sequence\s*=\s*hub_snapshot\.icm45686\.sequence') {
    $failures.Add('ICM V4 live payloads must preserve acquisition timestamp and sequence.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'PASS: master streams fresh BNO and ICM samples independently'

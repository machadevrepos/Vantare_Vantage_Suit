$ErrorActionPreference = 'Stop'

$centralPath = Join-Path $PSScriptRoot '..\Core\Src\ble\exo_hub_central_client.cpp'
$centralSource = Get-Content -Raw -LiteralPath $centralPath
$failures = [System.Collections.Generic.List[string]]::new()

function Get-FunctionBlock([string]$signature, [string]$nextSignature) {
    $start = $centralSource.IndexOf($signature)
    $end = $centralSource.IndexOf($nextSignature, $start + $signature.Length)
    if ($start -lt 0 -or $end -le $start) {
        throw "Unable to isolate $signature"
    }
    return $centralSource.Substring($start, $end - $start)
}

$setTiming = Get-FunctionBlock `
    'void exo_hub_central_client_set_transfer_timing' `
    'uint8_t exo_hub_central_client_transfer_preparation_resolved'
$preparationResolved = Get-FunctionBlock `
    'uint8_t exo_hub_central_client_transfer_preparation_resolved' `
    'void exo_hub_central_client_set_ble_ready'

if ($setTiming -notmatch 'exo_leaf_slot_node_id\([^\)]*\)\s*==\s*node_id') {
    $failures.Add('Transfer timing must tune only the selected upload source.')
}
if ($centralSource.Contains('begin_park_preparation')) {
    $failures.Add('Inactive-link parking must not be part of upload preparation.')
}
if ($preparationResolved.Contains('interval_profile_resolved')) {
    $failures.Add('Initial upload credit must not depend on inactive-link interval profiles.')
}
if ($preparationResolved -notmatch 'exo_leaf_slot_node_id\([^\)]*\)\s*!=\s*node_id') {
    $failures.Add('Preparation readiness must select the active Node only.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'Active-Node transfer timing source contract passed.'

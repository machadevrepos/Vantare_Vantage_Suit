$ErrorActionPreference = 'Stop'

$sourcePath = Join-Path $PSScriptRoot '..\STM32_WPAN\App\exo_hub_central_client.c'
$source = Get-Content -Raw $sourcePath
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Source([string]$Pattern, [string]$Message) {
    if ($source -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Source `
    'static\s+uint8_t\s+exo_should_resume_scan_after_timeout\s*\(\s*void\s*\)' `
    'Missing the discovery timeout policy that protects active phone streaming.'

Require-Source `
    'APP_BLE_LeafClientPhoneConnected\(\)\s*!=\s*0U[\s\S]*?return\s+0U' `
    'Discovery must not automatically restart while the phone is connected.'

Require-Source `
    'exo_ready_or_connecting_leaf_count\(\)\s*!=\s*0U[\s\S]*?return\s+0U' `
    'Discovery must not automatically restart after at least one leaf is connected.'

Require-Source `
    'if\s*\(\s*exo_should_resume_scan_after_timeout\(\)\s*!=\s*0U\s*\)' `
    'The GAP completion handler must apply the streaming-safe discovery policy.'

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'PASS: BLE leaf discovery does not repeatedly interrupt active streaming'

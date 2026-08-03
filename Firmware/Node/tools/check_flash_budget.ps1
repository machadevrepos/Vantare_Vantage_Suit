param(
    [string]$MapFile = "Node/Debug/Node.map",
    [int]$FlashBudgetBytes = 126976,
    [int]$WarnThresholdBytes = 124928,
    [int]$TopCount = 20
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $MapFile)) {
    throw "Map file not found: $MapFile"
}

$raw = Get-Content -LiteralPath $MapFile

$regionLength = $null
$flashUsed = $null
$inFlashSection = $false
$sizes = @{}

foreach ($line in $raw) {
    if ($line -match '^\s*FLASH\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)') {
        $regionLength = [Convert]::ToInt32($Matches[1], 16)
        continue
    }
    if ($line -match '^\.(isr_vector|text|rodata|ARM\.extab|ARM)\s+0x([0-9a-fA-F]+)\s+0x([0-9a-fA-F]+)') {
        $addr = [Convert]::ToInt64($Matches[2], 16)
        $size = [Convert]::ToInt64($Matches[3], 16)
        if ($addr -ge 0x08000000 -and $addr -lt 0x08000000 + 0x200000) {
            if ($null -eq $flashUsed) { $flashUsed = 0 }
            $flashUsed += $size
        }
    }
    if ($line -match '^\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+(.+\.(o|a)\S*)$') {
        $size = [Convert]::ToInt64($Matches[1], 16)
        $obj = $Matches[2].Trim()
        if (-not $sizes.ContainsKey($obj)) { $sizes[$obj] = 0L }
        $sizes[$obj] += $size
    }
}

if ($null -eq $flashUsed) {
    throw "Unable to parse FLASH usage from $MapFile"
}

$budget = $FlashBudgetBytes
if ($regionLength -and $regionLength -lt $budget) {
    $budget = $regionLength
}

$delta = $budget - $flashUsed
$pct = [math]::Round(($flashUsed * 100.0) / $budget, 2)

Write-Host "FLASH usage: $flashUsed / $budget bytes ($pct`%)"
if ($regionLength) {
    Write-Host "FLASH region length from map: $regionLength bytes"
}
Write-Host "Remaining: $delta bytes"

Write-Host ""
Write-Host "Top contributors:"
$sizes.GetEnumerator() |
    Sort-Object Value -Descending |
    Select-Object -First $TopCount |
    ForEach-Object { "{0,8}  {1}" -f $_.Value, $_.Key } |
    Write-Host

if ($flashUsed -gt $budget) {
    throw "FLASH budget exceeded by $(-$delta) bytes"
}
if ($flashUsed -gt $WarnThresholdBytes) {
    Write-Warning "FLASH usage above warning threshold ($WarnThresholdBytes bytes)"
}

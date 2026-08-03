$ErrorActionPreference = 'Stop'

$sourcePath = Join-Path $PSScriptRoot '..\..\LIBRARY\CUSTOM\BNO85_STM32.h'
$source = Get-Content -Raw $sourcePath
$failures = [System.Collections.Generic.List[string]]::new()

if ($source -notmatch 'sh2_setSensorConfig\s*\(\s*SH2_GAME_ROTATION_VECTOR\s*,') {
    $failures.Add('BNO85 must enable Game Rotation Vector for the exported relative orientation.')
}

if ($source -match 'sh2_setSensorConfig\s*\(\s*SH2_ROTATION_VECTOR\s*,') {
    $failures.Add('BNO85 must not enable Rotation Vector alongside Game Rotation Vector.')
}

$rotationCase = [regex]::Match(
    $source,
    'case\s+SH2_ROTATION_VECTOR\s*:(?<body>[\s\S]*?)\bbreak\s*;'
)
if ($rotationCase.Success -and
    $rotationCase.Groups['body'].Value -match 'latest_quat_|has_rotation_') {
    $failures.Add('Rotation Vector callbacks must not overwrite the exported Game Rotation Vector quaternion.')
}

if ($source -notmatch
    'case\s+SH2_GAME_ROTATION_VECTOR\s*:[\s\S]*?latest_quat_i_[\s\S]*?latest_quat_j_[\s\S]*?latest_quat_k_[\s\S]*?latest_quat_real_[\s\S]*?has_rotation_\s*=\s*true') {
    $failures.Add('Game Rotation Vector callbacks must populate the exported quaternion.')
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Output 'PASS: BNO85 exports orientation from Game Rotation Vector only'

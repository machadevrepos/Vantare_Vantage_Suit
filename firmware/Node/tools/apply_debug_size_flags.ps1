param(
    [string]$DebugDir = "Node/Debug"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $DebugDir)) {
    throw "Debug directory not found: $DebugDir"
}

$subdirFiles = Get-ChildItem -Path $DebugDir -Recurse -Filter "subdir.mk"
if ($subdirFiles.Count -eq 0) {
    throw "No subdir.mk files found under $DebugDir"
}

foreach ($file in $subdirFiles) {
    $content = Get-Content -LiteralPath $file.FullName -Raw

    # Normalize optimization for recurring debug-size builds.
    $content = $content -replace '\s-O0\s', ' -Os '

    # Apply C++ size flags once where g++ compile rules exist.
    if ($content -notmatch '-fno-exceptions') {
        $content = $content -replace '(arm-none-eabi-g\+\+[^`r`n]*?-fdata-sections)', '$1 -fno-exceptions -fno-rtti'
    }

    Set-Content -LiteralPath $file.FullName -Value $content -NoNewline
}

$mk = Join-Path $DebugDir "makefile"
if (Test-Path -LiteralPath $mk) {
    $m = Get-Content -LiteralPath $mk -Raw
    if ($m -notmatch '--print-memory-usage') {
        $m = $m -replace '(-Wl,--gc-sections)', '$1 -Wl,--print-memory-usage'
        Set-Content -LiteralPath $mk -Value $m -NoNewline
    }
}

Write-Host "Applied Debug-Size flags in $DebugDir"

param(
    [string]$ProjectRoot = ".",
    [string]$DebugDir = "Node/Debug"
)

$ErrorActionPreference = "Stop"

Push-Location $ProjectRoot
try {
    & powershell -NoProfile -ExecutionPolicy Bypass -File "Node/tools/apply_debug_size_flags.ps1" -DebugDir $DebugDir
    & make -C $DebugDir -j16 all
    & powershell -NoProfile -ExecutionPolicy Bypass -File "Node/tools/check_flash_budget.ps1" -MapFile "$DebugDir/Node.map"
}
finally {
    Pop-Location
}

# Syntax-checks the modified firmware C/C++ source files against the real ARM toolchain.
$armBin = 'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin'
$gcc   = Join-Path $armBin 'arm-none-eabi-gcc.exe'
$gpp   = Join-Path $armBin 'arm-none-eabi-g++.exe'

if (-not (Test-Path $gcc)) {
    Write-Error "ARM GCC not found at $gcc"
    exit 2
}

$repo = Resolve-Path (Join-Path $PSScriptRoot '..\..')
$failed = 0

# -----------------------------------------------------------------------
# Master: custom_stm.c
# -----------------------------------------------------------------------
$masterRoot = Join-Path $repo 'Firmware\Master'
$masterIncludes = @(
    "-I$(Join-Path $masterRoot 'Core\Inc')",
    "-I$(Join-Path $masterRoot 'STM32_WPAN\App')",
    "-I$(Join-Path $masterRoot 'Drivers\STM32WBxx_HAL_Driver\Inc')",
    "-I$(Join-Path $masterRoot 'Drivers\STM32WBxx_HAL_Driver\Inc\Legacy')",
    "-I$(Join-Path $masterRoot 'Utilities\lpm\tiny_lpm')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread\tl')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread\shci')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\utilities')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble\core')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble\core\auto')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble\core\template')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble\svc\Inc')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble\svc\Src')",
    "-I$(Join-Path $masterRoot 'Drivers\CMSIS\Device\ST\STM32WBxx\Include')",
    "-I$(Join-Path $masterRoot 'Utilities\sequencer')",
    "-I$(Join-Path $masterRoot 'Middlewares\ST\STM32_WPAN\ble')",
    "-I$(Join-Path $masterRoot 'Drivers\CMSIS\Include')",
    "-I$(Join-Path $repo 'Firmware\LIBRARY\CUSTOM')",
    "-I$(Join-Path $repo 'Firmware\LIBRARY\CUSTOM\motion.mcu.icm45686.driver\icm45686')",
    "-I$(Join-Path $masterRoot 'FATFS\Target')",
    "-I$(Join-Path $masterRoot 'FATFS\App')",
    "-I$(Join-Path $masterRoot 'Middlewares\Third_Party\FatFs\src')"
)
$masterCFlags = '-mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32WB55xx -fsyntax-only -Wall --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb'

$customStm = Join-Path $masterRoot 'STM32_WPAN\App\custom_stm.c'
$procArgs = $masterCFlags.Split(' ') + $masterIncludes + @($customStm)
$pinfo = New-Object System.Diagnostics.ProcessStartInfo
$pinfo.FileName = $gcc
$pinfo.Arguments = $procArgs -join ' '
$pinfo.RedirectStandardError = $true
$pinfo.RedirectStandardOutput = $true
$pinfo.UseShellExecute = $false
$p = [System.Diagnostics.Process]::Start($pinfo)
$stdout = $p.StandardOutput.ReadToEnd()
$stderr = $p.StandardError.ReadToEnd()
$p.WaitForExit()
if ($p.ExitCode -eq 0) {
    Write-Output "PASS Firmware/Master/STM32_WPAN/App/custom_stm.c"
} else {
    $failed++
    Write-Output "FAIL Firmware/Master/STM32_WPAN/App/custom_stm.c"
    ($stdout + $stderr) | ForEach-Object { Write-Output "     $_" }
}

# -----------------------------------------------------------------------
# Node: main.c  (compiled as C++ per .cproject)
# -----------------------------------------------------------------------
$nodeRoot = Join-Path $repo 'Firmware\Node'
$nodeIncludes = @(
    "-I$(Join-Path $nodeRoot 'Core\Inc')",
    "-I$(Join-Path $nodeRoot 'STM32_WPAN\App')",
    "-I$(Join-Path $nodeRoot 'Drivers\STM32WBxx_HAL_Driver\Inc')",
    "-I$(Join-Path $nodeRoot 'Drivers\STM32WBxx_HAL_Driver\Inc\Legacy')",
    "-I$(Join-Path $nodeRoot 'Utilities\lpm\tiny_lpm')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread\tl')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\interface\patterns\ble_thread\shci')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\utilities')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble\core')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble\core\auto')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble\core\template')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble\svc\Inc')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble\svc\Src')",
    "-I$(Join-Path $nodeRoot 'Drivers\CMSIS\Device\ST\STM32WBxx\Include')",
    "-I$(Join-Path $nodeRoot 'Utilities\sequencer')",
    "-I$(Join-Path $nodeRoot 'Middlewares\ST\STM32_WPAN\ble')",
    "-I$(Join-Path $nodeRoot 'Drivers\CMSIS\Include')",
    "-I$(Join-Path $repo 'Firmware\LIBRARY\CUSTOM')",
    "-I$(Join-Path $repo 'Firmware\LIBRARY\CUSTOM\motion.mcu.icm45686.driver\icm45686')"
)
$nodeCppFlags = '-mcpu=cortex-m4 -std=gnu++17 -DUSE_HAL_DRIVER -DSTM32WB55xx -fsyntax-only -Wall --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb'

$nodeMain = Join-Path $nodeRoot 'Core\Src\main.c'
$procArgs = $nodeCppFlags.Split(' ') + $nodeIncludes + @($nodeMain)
$pinfo2 = New-Object System.Diagnostics.ProcessStartInfo
$pinfo2.FileName = $gpp
$pinfo2.Arguments = $procArgs -join ' '
$pinfo2.RedirectStandardError = $true
$pinfo2.RedirectStandardOutput = $true
$pinfo2.UseShellExecute = $false
$p2 = [System.Diagnostics.Process]::Start($pinfo2)
$stdout2 = $p2.StandardOutput.ReadToEnd()
$stderr2 = $p2.StandardError.ReadToEnd()
$p2.WaitForExit()
if ($p2.ExitCode -eq 0) {
    Write-Output "PASS Firmware/Node/Core/Src/main.c"
} else {
    $failed++
    Write-Output "FAIL Firmware/Node/Core/Src/main.c"
    Write-Output ($stdout2 + $stderr2)
}

if ($failed -eq 0) {
    Write-Output "Firmware syntax checks: all passed."
} else {
    Write-Output "$failed firmware syntax check(s) failed."
    exit 1
}

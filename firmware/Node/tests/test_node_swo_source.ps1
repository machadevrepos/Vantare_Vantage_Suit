$ErrorActionPreference = 'Stop'

$nodeRoot = Split-Path -Parent $PSScriptRoot
$mainPath = Join-Path $nodeRoot 'Core\Src\main.c'
$appDebugPath = Join-Path $nodeRoot 'Core\Src\app_debug.c'
$traceHeaderPath = Join-Path $nodeRoot 'Core\Inc\node_swo_trace.h'

if (-not (Test-Path -LiteralPath $traceHeaderPath)) {
    throw "Missing Node SWO trace interface: $traceHeaderPath"
}

$main = Get-Content -Raw -LiteralPath $mainPath
$appDebug = Get-Content -Raw -LiteralPath $appDebugPath
$traceHeader = Get-Content -Raw -LiteralPath $traceHeaderPath

$requiredHeaderTokens = @(
    'void NodeSwo_Init(uint32_t core_clock_hz, uint32_t swo_clock_hz);',
    'uint8_t NodeSwo_TryWrite(uint8_t value);',
    'size_t NodeSwo_Write(const uint8_t *data, size_t size);',
    'void NodeSwo_Process(void);',
    'void NodeSwo_Logf(const char *format, ...);'
)

foreach ($token in $requiredHeaderTokens) {
    if (-not $traceHeader.Contains($token)) {
        throw "Node SWO header is missing required interface: $token"
    }
}

$requiredMainTokens = @(
    '#include "node_swo_trace.h"',
    'NodeSwo_Init(HAL_RCC_GetHCLKFreq(), 2000000U);',
    '[SWO][NODE] boot',
    '[SWO][NODE] alive',
    'NodeSwo_TryWrite',
    'NodeSwo_Write',
    'NodeSwo_Process',
    'g_node_swo_buffer',
    'kNodeSwoMaxWriteSize = 256U',
    'if (size > kNodeSwoMaxWriteSize)',
    'ITM->PORT[0U].u8',
    'GPIO_AF0_JTD_TRACE',
    'CoreDebug_DEMCR_TRCENA_Msk',
    'DBGMCU_CR_TRACE_IOEN',
    'TPI->SPPR = 2U'
)

foreach ($token in $requiredMainTokens) {
    if (-not $main.Contains($token)) {
        throw "Node main is missing required SWO behavior: $token"
    }
}

if ($main.Contains('#include <EXO_LOGGER.h>')) {
    throw 'Node main still routes EXO_LOG through the shared blocking SWO_PRINTER path'
}

if ($main -notmatch '#define\s+EXO_LOG\(\.\.\.\)\s+NodeSwo_Logf\(__VA_ARGS__\)') {
    throw 'Node EXO_LOG is not routed through the nonblocking Node SWO logger'
}

$swoInitStart = $main.IndexOf('NodeSwo_Init(HAL_RCC_GetHCLKFreq(), 2000000U);')
$sysInitStart = $main.IndexOf('/* USER CODE BEGIN SysInit */')
$sysInitEnd = $main.IndexOf('/* USER CODE END SysInit */')
if (($swoInitStart -lt $sysInitStart) -or ($swoInitStart -gt $sysInitEnd)) {
    throw 'Node SWO startup is not protected by the CubeMX SysInit user-code block'
}

$tryWriteStart = $main.IndexOf('extern "C" uint8_t NodeSwo_TryWrite(')
$writeStart = $main.IndexOf('extern "C" size_t NodeSwo_Write(')
if (($tryWriteStart -lt 0) -or ($writeStart -le $tryWriteStart)) {
    throw 'Node SWO writer functions are missing or out of order'
}
$tryWriteFunction = $main.Substring($tryWriteStart, $writeStart - $tryWriteStart)
if ($tryWriteFunction -match '\b(while|for)\s*\(') {
    throw 'NodeSwo_TryWrite contains a wait loop that can stall the firmware'
}

$dbgFunctionStart = $appDebug.IndexOf('void DbgOutputTraces(')
if ($dbgFunctionStart -lt 0) {
    throw 'DbgOutputTraces function not found'
}

$dbgFunction = $appDebug.Substring($dbgFunctionStart)
foreach ($token in @(
    '#include "node_swo_trace.h"',
    'if (CFG_DEBUG_TRACE_UART == 0)',
    'NodeSwo_Write(p_data, size)',
    'if (cb != NULL)',
    'HW_UART_Transmit_DMA(CFG_DEBUG_TRACE_UART, p_data, size, cb)'
)) {
    if (-not $appDebug.Contains($token) -and -not $dbgFunction.Contains($token)) {
        throw "Node debug trace routing is missing: $token"
    }
}

if ($dbgFunction.Contains('ITM_SendChar(')) {
    throw 'Node DbgOutputTraces still uses the blocking CMSIS ITM_SendChar helper'
}

Write-Host 'PASS: Node SWO source contract'

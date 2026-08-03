# Node Flash Budget Policy

## Budget
- `FLASH_BUDGET_BYTES = 126976` (124 KB)
- Warning threshold: `124928` bytes (122 KB)
- Source of truth: `Node/STM32WB55CCUX_FLASH.ld` (`FLASH LENGTH = 124K`)

## Required Build Profile
- Use size-safe debug flags for recurring development builds:
- `-Os`
- `-ffunction-sections -fdata-sections`
- `-fno-exceptions -fno-rtti` (for C++ compile units)
- Linker GC: `-Wl,--gc-sections`

## Feature Profiles
- `EXO_PROFILE_MINIMAL=1` (default): disables non-essential diagnostics.
- `EXO_PROFILE_DIAG=1`: enables diagnostic-heavy paths (use only when needed).

## Standard Command
Run this before commit and after meaningful feature edits:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File Node/tools/build_debug_size.ps1
```

## What the Command Does
1. Applies Debug size flags to generated `Node/Debug/*subdir.mk`.
2. Builds `Node/Debug`.
3. Parses `Node.map` and reports:
   - Used FLASH / budget / remaining
   - Top object/library contributors
4. Fails if FLASH exceeds budget.

## CI/Automation Recommendation
- Add `Node/tools/build_debug_size.ps1` as a required check for merge.
- Treat budget overflow as blocking.

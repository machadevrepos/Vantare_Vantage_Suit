# Vantare Vantage Suit

Firmware + host tooling for the Vantare Vantage Suit exoskeleton: one **Master**
hub and up to four sensor **Nodes** on an STM32WB55 BLE mesh, recording IMU
sessions to SD/flash and streaming live previews to a browser desktop tool.

```
+----------+  BLE central   +---------+   BLE peripheral   +---------+
|  Browser |<-------------->| Master  |<------------------>|  Nodes  |
| (web UI) |                | STM32WB55|    hub-leaf mesh  |x4 WB55  |
+----------+                 +---------+                    +---------+
     ^                            |  SD card                     | W25Q256
     |        CSV / binary        v                              v
     +------------------------ sessions <--------------------+
```

## Repository layout

| Path                  | Contents                                                        |
|-----------------------|-----------------------------------------------------------------|
| `firmware/master/`    | Master CubeIDE project (`Core/`, `STM32_WPAN/`, `build` outputs) |
| `firmware/node/`      | Node CubeIDE project                                            |
| `firmware/common/`    | Header-only shared modules under `inc/exo/`, shared linker script |
| `firmware/third_party/` | Vendored drivers: w25qxx, icm45686-driver, sh2                |
| `host/tests/`         | Python invariant tests + C++ module tests (CMake/ctest)         |
| `host/desktop_tool/`  | Browser-based single-file tool + BIN→CSV converter              |
| `host/scripts/`       | Session analysis / BLE monitoring utilities                     |
| `hardware/{master,node}/{v1.0,v1.1}/` | Schematics, PCB, BOM, gerbers, STEP             |
| `docs/`               | Architecture docs & guides                                      |
| `patches/`            | Historical firmware patches                                     |
| `scripts/`            | setup / format / lint helpers                                   |
| `data/`               | Local session data (gitignored)                                 |

## Quick start

### Firmware (requires STM32CubeIDE)
1. Open STM32CubeIDE and import the projects from `firmware/master` and
   `firmware/node` (project files `.cproject`/`.project` sit at those roots).
2. Build Debug/Release. Both projects share the linker script at
   `firmware/common/build/linker/STM32WB55_FLASH.ld` (768K flash window for the
   1MB WB55 with SFSA=0xD0).
3. Shared headers resolve via relative include paths (`../../common/inc`,
   `../../third_party/*`) — no workspace-relative LIBRARY project required.

### Host tests
```bash
pwsh host/tests/run_tests.ps1          # Windows: pytest + cmake/ctest
./host/tests/run_tests.sh              # Linux/macOS
# or individually:
python -m pytest host/tests/python -v
cmake -S host/tests/cpp -B host/tests/cpp/build && \
  cmake --build host/tests/cpp/build && \
  ctest --test-dir host/tests/cpp/build --output-on-failure
```

### Desktop tool
Open `host/desktop_tool/Exoskeleton.html` in a browser; convert recordings with
`host/scripts/convert_bin_to_csv.sh`.

## Conventions

- C++17-style header-only modules; shared code is namespaced under `exo/`
  includes (`#include <exo/storage/session_transfer.h>`).
- Third-party drivers are vendored in-tree under `firmware/third_party/`.
- Formatting/linting: `scripts/format.ps1` (clang-format), `scripts/lint.ps1`
  (cppcheck). See `.clang-format` / `.clang-tidy`.
- Version lives in [`VERSION`](VERSION) (semver).

Further reading: [docs/README.md](docs/README.md).

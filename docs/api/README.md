# API Reference

The firmware's shared modules are header-only and live in
`firmware/common/inc/exo/`:

| Module      | Path                          | Purpose                                        |
|-------------|-------------------------------|------------------------------------------------|
| protocol    | `exo/protocol/`               | BLE pipe, record & session-control protocols   |
| ble         | `exo/ble/`                    | Hub/leaf BLE manager + central client bridge   |
| sensors     | `exo/sensors/`                | BNO85 / ICM45686 drivers, CSV formatters       |
| storage     | `exo/storage/`                | Flash/SD recorders, session stager & transfer  |
| recording   | `exo/recording/`              | Node recording app, live sample queue          |
| types       | `exo/types/`                  | Shared enums, C structs, recording types       |
| utils       | `exo/utils/`                  | Logger, string handling, pins, JSON, includer  |

Per-project BLE app layers (differ between Master and Node) live in each
project at `Core/Inc/exo/ble/`.

A generated Doxygen reference can be added here later by running Doxygen with
`firmware/common/inc` as input.

# Vantare Vantage Suit Firmware Architecture

## System roles

The firmware uses one **Master** and up to four commissioned **Nodes**.

- Source ID `0` is the Master.
- Source IDs `1` through `4` are Nodes.
- The browser communicates with the Master over the Master BLE peripheral link.
- The Master communicates with Nodes over BLE central links.
- No active RS485 recording or streaming transport remains.

## Live preview

Each device samples its local BNO85 and ICM45686 sensors. Nodes retain the latest pending value per sensor and forward preview samples to the Master. The Master services Node/sensor slots fairly and forwards the five-source BLE V2 stream to the browser. Recording remains independent of preview delivery.

The Master uses a shared round-robin pacing gate at 10 ms normally and 20 ms after BLE backpressure. With four ready Nodes, this preserves 40 ms and 80 ms per-source cadence respectively. A failed latest-value sample is retained, other ready sources remain eligible, and normal cadence resumes after a stable recovery period.

## Recording and transfer

The Master records its own full-rate sensor data to SD. Each Node records full-rate sensor data to local flash. At recording completion, the Master owns reliable sequential collection from Nodes 1 through 4 using manifests, ACK windows, NACK ranges, CRC validation, and VerifyOk completion. Browser connectivity is not required for an active Node upload to progress.

Validated Node binaries remain on Master SD before conversion. Transfer ownership is not replaced until the active source completes verification.

## Training CSV version 2

The Master converts validated source binaries into a single training CSV containing raw sensor values, source counts, loss indicators, payload CRCs, per-sensor sequence and timing fields, timestamp-quality flags, quaternion-derived roll/pitch/yaw using the intrinsic Z-Y-X yaw-pitch-roll convention, and vector magnitudes.

A `.TMP` file is renamed to `.CSV` only after all expected sources are committed. The `.OK` marker is created, synchronized, and closed before the session is reported as published.

## Hardware boundary

STM32CubeMX-generated initialization, sensor buses, GPIO assignments, flash/SD interfaces, power controls, and generic HAL callback signatures remain unchanged by the BLE-only cleanup. Both STM32CubeIDE projects and the complete Master/four-Node workflow must be validated on hardware before this cleanup branch is merged.

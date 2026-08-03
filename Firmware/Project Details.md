# Exoskeleton Project Architecture and Data Flow

This project is split into three major parts:

1. `Master/`
2. `Node/Core/` as the hardware-runtime entry layer for each remote node
3. `DesktopTools/` as the browser-side operator console and data viewer

Together they form a pipeline where remote nodes capture motion data, the master coordinates the system and bridges traffic to BLE, and the desktop tools control the system, receive data, reconstruct recordings, visualize signals, and export selected results.

## High-Level System View

At a system level, the project behaves like this:

- A `Node` is a remote sensing endpoint with its own STM32WB55, IMU sensors, RS485 transport, and optional flash storage for local recording.
- The `Master` is the central coordinator. It has its own sensors, manages BLE, talks to the nodes over RS485, records its own local data to SD, and acts as the gateway between RS485 and the browser UI.
- `DesktopTools` connects to the master over BLE using Web Bluetooth, sends commands, receives live IMU packets and recorded data, performs acknowledgements and recovery control, and turns raw payloads into plots and CSV exports.

The practical control path is:

`DesktopTools -> BLE -> Master -> RS485 -> Nodes`

The practical data return path is:

`Nodes/Master sensors -> local buffers/storage -> RS485 and/or BLE notifications -> DesktopTools decode/reassemble -> plots/log/export`

## What `Master/` Does

`Master/` is the main hub firmware. It is not just another sensor board. It is the coordinator, BLE server, recording bridge, retransmission controller, and data egress point for the whole system.

### Main responsibilities of the master

The master firmware is responsible for:

- Booting and initializing the STM32WB platform, clocks, GPIO, DMA, RTC, I2C, SPI, UART, RF, BLE stack, and SD/FATFS plumbing.
- Sampling its own onboard/local sensors through the `HubSensorTestApp` path.
- Managing the BLE GATT service and its characteristics for IMU streaming, command writes, command acknowledgements, recording data, and recovery/control traffic.
- Receiving commands from the browser over BLE and dispatching them into local actions or RS485 actions.
- Discovering and managing remote nodes on the RS485 network.
- Starting synchronized recordings across master and nodes.
- Recording the master’s own local data to SD card storage using the SD session recorder path.
- Receiving node recording traffic over RS485 and forwarding or bridging it toward BLE.
- Running the reliable recording protocol: manifests, chunk windows, acknowledgements, NACKs, retransmissions, verify, commit, pause/cancel, and resume-like resynchronization.
- Recovering UART DMA receive state when the RS485 receive side becomes busy or faulted.

### Role of `Master/Core/Src/main.c`

`Master/Core/Src/main.c` is the real orchestration center for the firmware-specific behavior. It does far more than normal Cube-generated startup code. Its user sections instantiate the application objects and implement the project-specific control logic.

Key things defined there:

- `HubSensorTestApp` for collecting local hub/master sensor samples.
- `MasterRecordingController` for RS485-side recording coordination with nodes.
- BLE send callbacks that wrap `Custom_APP_SendRecordFrame`.
- The RS485 LPUART DMA receive buffer and recovery logic.
- A full local recording state machine for master-owned recordings.
- Recovery queues for retransmission jobs.
- BLE command handlers through `exo_hub_ble_write(...)`.

### Master BLE role

The master is the BLE peripheral/server. The desktop browser connects only to the master, never directly to nodes.

The custom BLE service exposes at least these logical channels:

- `IMU`: live sensor stream notifications
- `CMD`: command write path from desktop to master
- `CMD_ACK`: command acknowledgement and command report notifications from master to desktop
- `RECORD`: recording manifests, chunks, done frames, and some control traffic
- `RECOVERY`: dedicated recovery/control channel when present

`Master/STM32_WPAN/App/custom_app.c` is the bridge between BLE events and the application logic:

- BLE writes on `CMD`, `RECORD`, and `RECOVERY` call `exo_hub_ble_write(...)`.
- The file also implements the outbound helpers:
  - `Custom_APP_SendImuFrame(...)`
  - `Custom_APP_SendCmdAck(...)`
  - `Custom_APP_SendCmdReport(...)`
  - `Custom_APP_SendCmdNotify(...)`
  - `Custom_APP_SendRecordFrame(...)`
  - `Custom_APP_SendRecoveryFrame(...)`

That file is important because it is the narrow BLE boundary. Below it is project logic; above it is the BLE stack and GATT transport.

### Master local recording behavior

The master is not only a bridge for node data. It also records its own local session data.

The local recording logic in `Master/Core/Src/main.c` maintains a state machine with phases such as:

- `Idle`
- `Capturing`
- `RecordDoneWait`
- `Manifest`
- `TransferActive`
- `TransferPaused`
- `Resync`
- `Verifying`
- `Finished`
- `Cancelled`
- `ErrorStoredCanResume`

This means the master behaves like a first-class recording source, not just a controller.

Its local recording flow is roughly:

1. Desktop sends `StartRecord`.
2. Master validates the request and suppresses duplicates/replays.
3. Master arms its own local recorder and also triggers remote node recording coordination.
4. Master captures local sensor data to SD-backed session storage.
5. When local capture is complete, it emits a `RecordDone`-style result with size/CRC/session metadata.
6. Desktop acknowledges the manifest and opens a receive credit window.
7. Master streams session chunks over BLE.
8. Desktop acknowledges forward progress or NACKs missing ranges.
9. Master retransmits missing chunks if needed.
10. Desktop verifies the final CRC and sends verify/commit control.
11. Master finalizes the transfer state.

### Master RS485 role

The master is the only device that talks both BLE and RS485 in the end-user path.

On the RS485 side, the master:

- Initializes `hlpuart1` receive-to-idle DMA.
- Monitors UART busy/error conditions.
- Aborts and restarts DMA reception when needed.
- Uses `MasterRecordingController` to coordinate node-side recording and chunk transport.
- Tracks discovered nodes and exposes node discovery status back to the desktop.

This makes the master a protocol translator and scheduler:

- BLE is packetized around GATT writes/notifications and browser timing.
- RS485 is packetized around UART/DMA transport and multi-node coordination.

The master absorbs these differences so the browser can work with a single device and a single control model.

### Master command handling

The function `exo_hub_ble_write(...)` in `Master/Core/Src/main.c` is one of the most important command dispatch points in the project.

It handles:

- `StartRecord`
- stream start/stop and interval control
- node commissioning commands such as set/get node ID
- rediscovery and discovered node reporting
- record reset state
- reliable recording control frames
- legacy chunk acknowledgements

The master also sends explicit command acknowledgements back through `CMD_ACK` so the desktop can tell whether a request was accepted, rejected, or resulted in a side-band report.

### Master data categories

The master sends multiple kinds of data upward:

- live IMU frames for visualization
- command acknowledgements and reports
- recording manifests
- recording chunks
- recovery/control protocol frames
- discovered node reports

This is why the desktop code keeps separate BLE characteristic listeners instead of using a single generic stream.

## What `Node/Core/` Does

`Node/Core/` is the hardware/application entry layer for a remote sensing node. It brings up the MCU, sensor buses, RS485 UART path, flash SPI path, and runtime services that the node-specific recording logic depends on.

Important distinction:

- `Node/Core/` is where hardware is initialized and the top-level runtime loop lives.
- The BLE stack and profile files still exist under `Node/STM32_WPAN/`, but the node’s practical project-specific role is defined by `Node/Core/Src/main.c`.

### Main responsibilities of a node

A node firmware instance is responsible for:

- Initializing local hardware peripherals.
- Powering sensors and bus interfaces.
- Initializing the optional flash-backed recording subsystem.
- Loading or applying the runtime node ID.
- Starting the RS485 recording responder.
- Sampling and storing node-local sensor data during a recording session.
- Answering RS485-side recording requests from the master.
- Returning recorded chunks or retransmissions when asked.

### Role of `Node/Core/Src/main.c`

This file defines the practical node behavior.

It instantiates:

- `NodeRecordingApp` when flash-backed recording is enabled
- `NodeRecordingResponder` for RS485 protocol handling
- optional sensor test helpers
- runtime configuration hooks for storage-backed node ID persistence

It also:

- powers the node hardware
- adjusts SPI prescaler for flash access
- probes/inites flash
- loads runtime node ID from persistent storage when available
- starts the RS485 responder
- runs the main processing loop

### Node recording storage

Node recording is built around local nonvolatile storage when enabled.

The `NodeRecordingApp` configuration includes:

- node ID
- sensor I2C addresses
- flash region start
- flash region size

This means a node does not stream every sample live to the desktop during a recording. Instead, for the reliable recording path, it can:

1. record locally
2. finish capture
3. announce completion
4. provide a manifest
5. send chunks on demand
6. retransmit missing chunks later

That design is important because it decouples high-rate capture from lower-rate BLE/browser delivery.

### Node runtime configuration

The node can load its runtime node ID from persistent storage instead of hard-wiring only the compile-time default.

The relevant behavior in `Node/Core/Src/main.c` is:

- flash storage hooks are installed
- runtime node ID is loaded
- the recording app gets that node ID
- the RS485 responder gets that node ID

This is how commissioning commands can persist identity across resets when flash is available.

### Node RS485 role

The node does not sit on the BLE front end in the same way the master does. Its operational identity in this system is as an RS485 recording endpoint.

The node responder:

- listens on `hlpuart1`
- reacts to receive complete / receive idle / receive error callbacks
- processes RS485 discovery and recording protocol frames
- starts local recording when requested
- returns data chunks or retransmissions

The callback wiring in `Node/Core/Src/main.c` makes that explicit:

- `HAL_UART_RxCpltCallback(...)`
- `HAL_UARTEx_RxEventCallback(...)`
- `HAL_UART_ErrorCallback(...)`

All three feed into `node_rs485_recording`.

### Node direct BLE behavior

There is still BLE framework code in the node project, but from the project behavior in `Node/Core/Src/main.c`, the node’s project-specific BLE write handling is limited compared with the master.

The exposed `exo_node_ble_write(...)` path mainly supports:

- `StartRecord`
- completion acknowledgement
- reset-to-idle-and-erase

That indicates the node is not intended to be the primary operator-facing BLE endpoint. The master is. The node’s main production path is RS485 responder + local storage + replay.

## What `DesktopTools/` Does

`DesktopTools/` is the operator console for this system. It is a browser-based toolset, not embedded firmware. It connects to the master via Web Bluetooth and acts as:

- control panel
- live telemetry viewer
- recording receiver
- protocol endpoint for ack/nack/recovery
- data decoder
- plotter
- selection/export tool

### Files in `DesktopTools/`

The important files are:

- `index.html`
- `android.html`
- `app.js`
- `styles.css`

`app.js` contains the operational logic. The HTML files embed or mirror the same control concepts for desktop and Android-oriented usage.

### BLE client role

`DesktopTools/app.js` defines the BLE contract the browser expects from the master:

- service UUID
- IMU characteristic UUID
- CMD characteristic UUID
- CMD_ACK characteristic UUID
- RECORD characteristic UUID
- RECOVERY characteristic UUID

It validates that the master exposes the right properties:

- IMU must notify
- CMD must support write without response
- CMD_ACK must notify
- RECORD must notify and support write/writeWithoutResponse
- RECOVERY is preferred when present

If `RECOVERY` is absent, the desktop falls back to using `RECORD` for control writes. This is visible both in the code and in `system logs.log`, where the tool logs:

- recovery characteristic missing
- fallback to legacy RECORD control writes

### Operator actions provided by DesktopTools

The UI supports:

- connect/disconnect
- start and stop live streaming
- set live stream interval
- start recording sessions
- set node ID
- query node ID
- rediscover nodes
- refresh discovered node list
- inspect transfer status
- inspect lane progress
- inspect packet/invalid/unknown counters
- browse traces
- select points on the plot
- export selected data to CSV
- copy logs

### Live telemetry handling

For live stream mode, the desktop:

1. connects to the master BLE service
2. enables notifications
3. sends `START_STREAM`, `STOP_STREAM`, or `SET_INTERVAL`
4. receives IMU notifications
5. decodes payloads by sensor type and format version
6. stores time-series points in memory
7. renders traces on the plot

The decoder table in `app.js` includes:

- BNO payload decoding
- ICM payload decoding
- FLEX payload decoding

Decoded signals are organized by:

- source/node ID
- sensor ID
- signal name

Then each trace becomes selectable and plottable.

### Recording receiver role

The browser is not passive during recording transfer. It participates in the protocol.

It does the following:

1. sends `STOP_STREAM`
2. sends record-state reset
3. sends `StartRecord`
4. waits for record completion and manifest information
5. opens a receive window with a manifest acknowledgement
6. tracks received chunks by session and source
7. detects duplicates, gaps, and missing ranges
8. sends forward acknowledgements or NACK ranges
9. requests retransmission when holes remain
10. verifies CRC when transfer completes
11. sends commit/complete style control when the received file verifies

This logic is why the desktop keeps transfer objects such as:

- active transfer state
- per-session maps
- completed transfer maps
- pending acknowledgement bookkeeping
- retry timing guards
- watchdog and guard timers

### Recording protocol handling in the desktop

The desktop code explicitly knows about:

- `RecordDone`
- `SessionChunk`
- `ChunkAck`
- `LaneFrameV3`
- `ReliableFrame`

It also knows about reliable frame subtypes such as:

- manifest
- manifest ack
- ack window
- NACK range
- chunk
- pause
- resume
- cancel
- verify ok
- verify fail
- busy-not-owner
- source waiting
- commit done

This tells us the browser is acting as a full transfer endpoint, not just a dumb downloader.

### Lane-based transfer model

The desktop tracks lane statistics for V3 recording transfer:

- control lane
- master data lane
- node data lane
- retransmit lane

That means the transfer path distinguishes between:

- control messages
- master-origin payload data
- node-origin payload data
- retransmitted recovery traffic

This separation is useful for debugging and throughput visibility because a slow or failing transfer can be understood by lane, not only by total bytes.

### Plotting and export role

Once payloads are decoded, the desktop:

- groups traces by node and sensor
- assigns colors
- clips each trace to a rolling point window
- renders axes and lines on a canvas
- lets the user toggle visibility
- lets the user click or drag-select points
- shows selected rows in a table
- exports selected points as CSV

So `DesktopTools` is both a protocol console and a lightweight signal analysis tool.

## End-to-End Data Flow

This section describes the important data paths in detail.

### 1. Live stream data flow

The live streaming path is:

1. User clicks `Connect` in `DesktopTools`.
2. Browser connects to the master BLE service.
3. Browser enables notifications on `IMU`, `CMD_ACK`, and `RECORD`, and optionally `RECOVERY`.
4. User clicks `Start`.
5. Browser sends `START_STREAM` over `CMD`.
6. Master accepts the command and starts periodic live stream behavior.
7. Master reads local sensor snapshots and packages them into IMU frames.
8. Master sends those frames through `Custom_APP_SendImuFrame(...)`.
9. Browser receives IMU notifications.
10. Browser decodes the payload based on sensor type and version.
11. Decoded samples are pushed into the in-memory trace store.
12. Plot and selection UI update.

Notes:

- This is mainly for operator feedback and debugging.
- It is not the same as the robust recorded-session transfer path.
- The point window is intentionally bounded so the browser remains responsive.

### 2. Recording start flow

The recording start path is:

1. User enters session parameters in the desktop UI.
2. Desktop sends `STOP_STREAM` first.
3. Desktop sends `RESET_RECORD_STATE`.
4. Desktop sends the binary `StartRecord` message.
5. Master receives the BLE write through `custom_app.c` and dispatches it to `exo_hub_ble_write(...)`.
6. Master validates:
   - message length
   - duplicate/replay conditions
   - current transfer/recording ownership state
7. Master arms its local recorder.
8. Master triggers the node-side recording control through the RS485 recording controller.
9. Master sends a `CMD_ACK` response back to the desktop.
10. Desktop marks the request as accepted and waits for record completion rather than assuming transfer begins immediately.

The logs in `system logs.log` reflect this staged behavior:

- start record ACK
- record flow armed
- waiting for `RECORD done` / manifest-driven transfer

### 3. Node-side capture flow

For a remote node:

1. Master sends a recording control request over RS485.
2. Node responder receives and parses it.
3. Node recording app starts local capture.
4. Node samples its configured sensors.
5. Node stores data into flash-backed session storage.
6. Node later reports completion metadata back through the master’s RS485 coordination path.

For the master’s own local capture:

1. Master starts its local recorder directly.
2. Hub sensors are sampled locally.
3. Samples are appended into SD-backed session data.
4. Master later advertises completion metadata as source `MASTER`.

### 4. Manifest and chunk transfer flow

After recording is complete, the transfer path becomes file-like.

The sequence is:

1. Source reports `RecordDone` / equivalent completion information.
2. Source sends a manifest:
   - session ID
   - source ID
   - total file size
   - chunk size
   - total chunks
   - CRC
   - duration
3. Desktop allocates/updates transfer tracking state.
4. Desktop sends manifest acknowledgement with receive credit.
5. Source streams chunks.
6. Desktop marks each chunk by index and byte range.
7. If chunks arrive in order and no holes exist, desktop advances `next_chunk_index`.
8. If holes exist or timeouts happen, desktop sends NACK ranges.
9. Source retransmits through the retransmit path/lane.
10. Once all chunks are present, desktop verifies CRC.
11. Desktop sends verify result and final commit/complete control.

### 5. Acknowledgement and recovery flow

This system has explicit recovery behavior rather than relying on best-effort BLE delivery.

Important mechanisms visible in code and logs:

- receive credit windows
- acknowledgement guard timing
- control heartbeats
- NACK retry guards
- retransmit queues
- stale ACK detection
- verification failures feeding retransmission
- queueing recovery jobs when overlap happens

On the master side, recovery is tracked with a bounded recovery job queue. Jobs can represent:

- master local retransmit
- node UART retransmit
- verify-stage retransmit

On the browser side, `app.js` maintains:

- pending acknowledgement state
- write busy / in-flight state
- watchdog timers
- retry timing
- transfer maps keyed by `sessionId:sourceId`

This means the reliable recording path is intentionally stateful on both ends.

### 6. Verification and commit flow

The transfer is only considered complete after data integrity is checked.

The final steps are:

1. Desktop receives all expected chunks.
2. Desktop computes or validates the expected payload CRC.
3. If CRC matches, desktop sends `VerifyOk`.
4. Master or node accepts verification and sends/handles `CommitDone`.
5. If CRC fails, desktop or firmware can issue `VerifyFail`, which feeds retransmission logic instead of silently accepting corruption.

This is a major difference between this project and a simple BLE file dump. The system is trying to guarantee usable session capture even across partial losses.

## Recording Media and Storage Roles

The project uses different storage roles for master and node.

### Master storage

The master uses SD/FATFS-related components and the `MasterSdSessionRecorder` path.

Its storage role is:

- persist the master’s own local captured session
- make that session replayable over BLE in reliable chunks
- survive temporary transport stalls during acquisition

### Node storage

The node uses flash-backed storage through the node recording app.

Its storage role is:

- persist node-local captured session data
- support delayed upload after capture
- support retransmission without resampling
- support node ID persistence and some runtime state

This split makes architectural sense:

- nodes need compact local storage and low-latency capture
- the master needs larger removable/session-oriented storage plus gateway duties

## What `system logs.log` Shows About Runtime Behavior

The log file is useful because it confirms the design intent seen in code.

It shows:

- BLE connection and notification setup
- fallback when `RECOVERY` characteristic is not available
- command acknowledgements arriving on `CMD_ACK`
- start-record handshake
- discovered node reports
- manifest reception for a master-owned recording
- periodic ACK/control behavior
- duplicate chunk detection
- backpressure cases such as `cmdWriteBusy`
- recovery conditions such as NACK retry guard behavior

So the logs are not just debug noise. They expose the runtime state machine of the recording transport.

## Division of Responsibility Summary

### `Master/`

`Master/` is the coordinator and bridge.

It:

- owns the BLE interface the browser talks to
- owns local master recording
- owns RS485 coordination with remote nodes
- owns command dispatch
- owns data forwarding upward
- owns a large part of recovery and retransmission orchestration

### `Node/Core/`

`Node/Core/` is the hardware/runtime shell of each node.

It:

- initializes the node hardware
- initializes flash-backed recording when enabled
- loads runtime node identity
- starts the RS485 responder
- captures node-local sensor sessions
- serves stored data back when the master requests it

### `DesktopTools/`

`DesktopTools/` is the operator-facing client.

It:

- connects to the master over BLE
- issues commands
- receives acknowledgements and status reports
- reconstructs recording sessions from manifests and chunks
- drives ACK/NACK/recovery control
- decodes sensor payloads
- plots and exports results

## Practical Interpretation

If this system is viewed as a layered stack, it looks like this:

- `DesktopTools`: operator UI, protocol client, plotting, export
- `Master`: BLE server, command gateway, local recorder, RS485 coordinator
- `Node`: distributed capture endpoint with flash-backed session replay

If it is viewed as a recording pipeline, it looks like this:

- capture on master and nodes
- store locally where needed
- announce completion with metadata
- transfer as chunked reliable sessions
- recover missing pieces
- verify integrity
- expose usable decoded signals in the browser

That is the core purpose of the project: coordinated multi-source exoskeleton sensor acquisition with reliable post-capture retrieval and operator-side visualization.

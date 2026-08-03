# Master + Four-Node Integration Test Plan

## Implemented branch behavior

`2nd_Branch` supports source IDs 0 through 4, where source 0 is the Master and
sources 1 through 4 are Node PCBs. The Master uses a deterministic Node transfer
order, tracks one active Node transfer owner, records ACK/NACK resume position,
and does not release active ownership until the reliable verification callback
completes.

The existing recording pipeline remains responsible for:

- synchronized prepare and commit across the selected Node mask;
- Master recording to SD;
- Node recording to external flash;
- sequential Node upload;
- SD staging and CRC validation;
- combined Master and Node training CSV generation;
- source-specific plotting in `Firmware/DesktopTools/Exoskeleton.html`.

## Host-test coverage

`Firmware/HostTests/test_master_node_transfer_window.cpp` verifies:

- invalid source rejection;
- contiguous chunk acceptance;
- exact duplicate recognition;
- forward-gap detection;
- corrupt-frame detection;
- final-chunk completion;
- transfer reset behavior.

Build command:

```sh
g++ -std=c++17 -Wall -Wextra -Werror \
  -I Firmware/LIBRARY/CUSTOM \
  Firmware/HostTests/test_master_node_transfer_window.cpp \
  -o test_master_node_transfer_window
./test_master_node_transfer_window
```

## Required hardware validation

1. Program four Nodes with unique IDs 1, 2, 3, and 4.
2. Program the Master from `2nd_Branch`.
3. Confirm the topology report contains exactly Nodes 1 through 4.
4. Start a 10-second session with selected Node mask `0x1E`.
5. Confirm every Node acknowledges Prepare before Commit is sent.
6. Confirm the Master and all Nodes stop and finalize the same session ID.
7. Confirm Node transfers occur in order 1, 2, 3, 4.
8. Confirm each source passes header CRC and payload CRC32 validation.
9. Confirm the completed source mask is `0x1F`.
10. Confirm the HTML page displays Master and Nodes 1 through 4 separately.

## Fault-injection validation

Repeat the test while introducing each failure independently:

- disconnect the browser during recording;
- disconnect the browser during Node upload;
- power-cycle one Node during upload and reconnect it;
- introduce RF interference;
- resend an already accepted chunk;
- request retransmission of a missing chunk;
- fill the SD card;
- remove or corrupt the SD card during staging.

A run is accepted only when no source is marked complete before CRC validation,
no different Node becomes the active transfer owner concurrently, and incomplete
sessions remain identifiable as incomplete rather than being published as valid.

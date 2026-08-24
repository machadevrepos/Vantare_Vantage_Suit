# Linux BLE Stop GATT Response Design

## Goal

Make the Master accept desktop Stop commands reliably through Chrome/BlueZ on
Linux without regressing the working Windows path or write-without-response
commands.

## Branch and scope

The change is based on `origin/ble-only-firmware-cleanup`, the active firmware
line containing 110 commits beyond `main`. It does not merge the divergent
`Testing_Hardware`-only commit or alter the Node firmware, recording state
machine, transfer protocol, or browser retry policy.

## Root cause

For an ATT Write Request, the generated Master service callback currently calls
`Custom_STM_App_Notification()` before `aci_gatt_permit_write()`. The application
callback processes Stop synchronously and forwards two commands to every active
Node. The desktop therefore waits for its ATT Write Response while the Master is
performing downstream BLE work. Chrome/BlueZ reports the outstanding operation
as `GATT Error Unknown`, and retries encounter `GATT operation already in
progress`.

Windows accepting the same timing does not make the ordering interoperable; it
only means its Bluetooth stack tolerates the delayed response.

## Design

In the Master's `ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE` handler for
`PipeControlRx`:

1. Populate the application notification with the acknowledged-write opcode
   `CUSTOM_STM_PIPECTRLRX_WRITE_EVT` and the received payload.
2. Call `aci_gatt_permit_write()` immediately and retain its status.
3. Call `Custom_STM_App_Notification()` only when the permit response succeeds.
4. Return from the custom handler so the generated duplicate block cannot
   process the same request a second time.

The existing `ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE` path remains responsible
for `CUSTOM_STM_PIPECTRLRX_WRITE_NO_RESP_EVT` and is unchanged.

## Error handling

If `aci_gatt_permit_write()` fails, the payload must not be dispatched into the
recording state machine because the client has not received a successful ATT
acceptance. The firmware continues servicing later BLE events; no retry is
performed inside the event callback.

## Verification

- Add a focused regression check that fails on the current notify-before-permit
  ordering and on use of the write-without-response opcode in the permit path.
- Run the existing BLE-only cleanup and remote lifecycle host guards.
- Run whitespace validation and any locally available firmware compilation.
- Hardware acceptance requires rebuilding/flashing the Master, reproducing one
  session on Fedora Chrome, and observing both a completed desktop command write
  and the Session Stop ACK. Windows remains a cross-platform regression check.


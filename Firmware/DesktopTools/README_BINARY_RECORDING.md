# Vantage Binary Recording Workflow

The shipping Master uses **binary-first recording**. The desktop page is a control, status, and live-preview console; it is not the authoritative recording destination.

## Client demo workflow

1. Flash the Master and Nodes from the same firmware commit.
2. Put a FAT/FAT32-formatted SD card in the Master and power the system normally.
3. Serve `Firmware/DesktopTools/Exoskeleton.html` from a local web server in Chrome or Edge, for example:

   ```bash
   cd Firmware/DesktopTools
   python -m http.server 8000
   ```

   Then open `http://localhost:8000/Exoskeleton.html`.
4. Connect to the Master, discover Nodes, select the participating Nodes, and confirm that the selected Nodes report ready/idle.
5. Start the recording. The browser may continue to show live preview data, but it does not ACK, reconstruct, or download the authoritative recording stream.
6. Stop the recording and wait for **Master SD Collection: Complete**. The Master first archives its own recording and then pulls each selected Node sequentially. A Node is only allowed to erase its local recording after the Master has validated and durably stored that Node file.
7. After the run is complete, power down before removing the SD card. A run uses a shared index:

   - `R####M.BIN` — Master
   - `R####N1.BIN` ... `R####N4.BIN` — selected Nodes

8. Copy the BIN files to the computer and convert them:

   ```bash
   python vantage_bin_to_csv.py /path/to/copied/session/files -o converted_csv
   ```

The converter validates the ESOX/v4 header, exact canonical file size, header CRC, payload CRC, sample counts, monotonic timestamps, and ICM sequence continuity before exporting anything. It intentionally rejects legacy sparse Master archives and incomplete/corrupt files.

For each valid BIN it writes:

- `<name>_BNO85.csv`
- `<name>_ICM45686.csv`
- `<name>_metadata.json`

The metadata file includes target/effective sample rates and firmware loss flags so a recorded demo can be checked before delivery.

## Demo acceptance checklist

Before recording the client video, run at least one short end-to-end session and confirm all of the following:

- Master and every selected Node transition out of Recording after Stop.
- Desktop reaches **Master SD Collection: Complete** without a Stage Error.
- Every selected source has exactly one `R####*.BIN` file with the same run index.
- `vantage_bin_to_csv.py` reports `OK` for every file in the run.
- No `R####T.BIN` temporary archive remains after a successful run.
- Nodes return to idle/ready after their validated files have been collected.
- The metadata JSON is reviewed for loss flags and achieved sample rates before presenting the recording as measurement-grade data.

If collection fails, do not start another recording until the retained Node data has been recovered or deliberately discarded. A failed Node transfer is designed to retain that Node's flash copy rather than erase unvalidated data.
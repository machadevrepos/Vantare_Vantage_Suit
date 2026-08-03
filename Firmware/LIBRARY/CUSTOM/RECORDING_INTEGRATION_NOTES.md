# Recording Integration Notes

## Manual CubeMX / CubeIDE work still required

- Keep `BNO85` on `I2C3` and `ICM-45686` on `I2C1`.
- Configure the node-side SPI instance and chip-select GPIO for the `W25Q256JVEIQ` flash before instantiating `W25Q256Flash`.
- Reconfigure hub `SPI1` manually for SD-card-compatible 8-bit SPI operation; the current generated project uses 4-bit data with hardware NSS.
- Add FatFs middleware to the hub project manually before using `HubSessionStore`; the current project has no generated FatFs files.
- Confirm the external pull-up strategy for `I2C3` (`PA7` / `PB4`) because the current generated setup does not enable internal pull-ups.
- Add the vendored C driver sources you actually use to each node project's build:
  - `sh2/*.c`
  - `motion.mcu.icm45686.driver/icm45686/imu/*.c`
  - `w25qxx/src/driver_w25qxx.c`
- Add include roots for `LIBRARY/CUSTOM` and `LIBRARY/CUSTOM/motion.mcu.icm45686.driver/icm45686`.
- Runtime firmware baseline uses `LIBRARY/CUSTOM/w25qxx` (LibDriver). `LIBRARY/CUSTOM/W25Qxxx_SPI_FLASH_STM32` is retained only for external-loader/tooling workflows.

## Integration shape

- `NodeRecorder` owns session state and flash layout.
- `NodeRecordingApp` owns the node-side sensor/flash composition and recording lifecycle.
- `HubRecordingApp` owns hub-side session assembly and SD persistence.
- `HubSensorTestApp` starts both hub IMUs at boot, prints samples, and writes `/SESSIONS/hub_sensor_test.csv` when FatFs disk I/O is functional.
- `SessionUploadReader` presents the split flash regions as one logical byte stream for BLE chunking and resume.
- `HubSessionAssembler` validates uploaded chunks before they are committed to SD.
- `Bno85Stm32`, `Icm45686Stm32`, and `W25Q256Flash` are reusable hardware adapters.
- `BLE_RECORD_PROTOCOL.h` defines the transport-neutral binary messages used over the existing BLE path.
- `HubSessionStore` persists complete uploaded node sessions to SD once FatFs is available.

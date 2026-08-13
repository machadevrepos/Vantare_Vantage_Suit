#ifndef EXO_HOST_APP_FATFS_H_
#define EXO_HOST_APP_FATFS_H_

/* Host-test shim. The firmware's app_fatfs.h drags in the STM32 HAL, which is
 * not available (and not relevant) when the storage-facing modules are exercised
 * against the FatFs stub. Everything the tests need comes from ff.h. */
#include "ff.h"

/* Mirrors the volume handles the firmware's app_fatfs.h exports. Declarations
 * are enough for the -fsyntax-only suites; no test links against them. */
extern FATFS USERFatFs;
extern FIL USERFile;
extern char USERPath[4];

#endif

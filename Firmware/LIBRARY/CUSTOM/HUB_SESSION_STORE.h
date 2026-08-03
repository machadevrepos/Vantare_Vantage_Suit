#ifndef HUB_SESSION_STORE_H_
#define HUB_SESSION_STORE_H_

#include <stdint.h>
#include <stdio.h>

#include "RECORDING_TYPES.h"

#if __has_include(<ff.h>)
extern "C" {
#include <ff.h>
}
#define EXO_HAS_FATFS 1
#elif __has_include(<FatFs/ff.h>)
extern "C" {
#include <FatFs/ff.h>
}
#define EXO_HAS_FATFS 1
#else
#define EXO_HAS_FATFS 0
#endif

namespace exo {

class HubSessionStore {
public:
#if EXO_HAS_FATFS
    static bool write_session_file(const char *root_dir, const SessionHeader &header, const uint8_t *payload) {
        char path[96] = {0};
        snprintf(path, sizeof(path), "%s/node_%u_session_%lu.bin",
                 root_dir,
                 static_cast<unsigned>(header.node_id),
                 static_cast<unsigned long>(header.session_id));

        FIL file;
        UINT written = 0U;
        const uint32_t payload_size = header.bno85_payload_size + header.icm45686_payload_size;
        if (f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
            return false;
        }
        const bool ok =
            f_write(&file, &header, sizeof(header), &written) == FR_OK &&
            written == sizeof(header) &&
            f_write(&file, payload, payload_size, &written) == FR_OK &&
            written == payload_size;
        f_close(&file);
        return ok;
    }
#else
    static bool write_session_file(const char *, const SessionHeader &, const uint8_t *) {
        return false;
    }
#endif
};

} // namespace exo

#endif

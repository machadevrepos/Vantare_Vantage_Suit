#ifndef HUB_RECORDING_APP_H_
#define HUB_RECORDING_APP_H_

#include <string.h>

#include "HUB_SESSION_STORE.h"
#include "SESSION_TRANSFER.h"

namespace exo {

class HubRecordingApp {
public:
    static constexpr uint32_t kMaxPayloadSize = 64U * 1024U;

    bool begin(const char *root_dir) {
        if (root_dir == nullptr) {
            return false;
        }
        strncpy(root_dir_, root_dir, sizeof(root_dir_) - 1U);
#if EXO_HAS_FATFS
        if (f_mount(&fatfs_, "", 1U) != FR_OK) {
            return false;
        }
        const FRESULT mkdir_result = f_mkdir(root_dir_);
        return mkdir_result == FR_OK || mkdir_result == FR_EXIST;
#else
        return false;
#endif
    }

    bool begin_session(const SessionHeader &header) {
        return assembler_.begin(header, payload_, sizeof(payload_));
    }

    bool append_chunk(uint32_t payload_offset, const uint8_t *data, uint32_t size) {
        return assembler_.append_payload_chunk(payload_offset, data, size);
    }

    bool finalize_session() {
        return assembler_.complete() &&
               HubSessionStore::write_session_file(root_dir_, assembler_.header(), payload_);
    }

    uint32_t next_offset() const { return assembler_.next_offset(); }

private:
#if EXO_HAS_FATFS
    FATFS fatfs_{};
#endif
    char root_dir_[32] = "/SESSIONS";
    uint8_t payload_[kMaxPayloadSize]{};
    HubSessionAssembler assembler_{};
};

} // namespace exo

#endif

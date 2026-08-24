#ifndef MASTER_BINARY_SESSION_INDEX_H_
#define MASTER_BINARY_SESSION_INDEX_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_fatfs.h"
#include "ff.h"

namespace exo {

class MasterBinarySessionIndex {
public:
    static constexpr uint16_t kMaxFileIndex = 9999U;

    /* Boot-time cache warm-up (call once after FATFS init and SD power-up,
     * outside any BLE context). Reads the persisted marker; if it is missing
     * or corrupt, enumerates /SESSIONS once and (re)writes the marker. The
     * enumeration is a single directory pass — bounded and fast even on a
     * card full of sessions. Tolerates a missing card: allocate() re-tries
     * lazily. */
    void init_cache()
    {
        (void) load_cache();
    }

    bool allocate(uint16_t &out_index)
    {
        out_index = 0U;
        if (!ensure_fs()) {
            return false;
        }
        if (!cache_ready_ && !load_cache()) {
            return false;
        }

        /* Marker semantics: the marker is persisted the moment an index is
         * handed out, so a crash between allocation and archive strands the
         * index (no file) but never re-issues it. Only skip forward. */
        for (uint16_t index = cached_last_ + 1U; index <= kMaxFileIndex; ++index) {
            bool occupied = false;
            if (!index_occupied(index, occupied)) return false;
            if (occupied) continue;
            if (!persist_marker(index)) return false;
            cached_last_ = index;
            out_index = index;
            last_result_ = FR_OK;
            return true;
        }
        last_result_ = FR_EXIST;
        return false;
    }

    FRESULT last_result() const { return last_result_; }

private:
    static constexpr uint32_t kMarkerMagic = 0x58444E49U; /* "INDX" */

    /* Recognize "R####<suffix>" and "TRN####<suffix>" names (8.3 short names
     * as produced by make_path/make_staging_path). Anything else — including
     * RUNIDX.BIN and MREC.BIN — must not advance the cached index. */
    static bool parse_session_index(const char *name, uint16_t &out_index)
    {
        if (name == nullptr) return false;
        size_t digits_at = 0U;
        if (name[0] == 'T' && name[1] == 'R' && name[2] == 'N') {
            digits_at = 3U;
        } else if (name[0] == 'R') {
            digits_at = 1U;
        } else {
            return false;
        }
        uint32_t value = 0U;
        for (uint8_t d = 0U; d < 4U; ++d) {
            const char c = name[digits_at + d];
            if (c < '0' || c > '9') return false;
            value = value * 10U + static_cast<uint32_t>(c - '0');
        }
        const char next = name[digits_at + 4U];
        const bool suffix_ok = (digits_at == 1U) ?
                (next == 'M' || next == 'N') :   /* R####M.BIN / R####N#.BIN */
                (next == '.');                    /* TRN####.CSV/.OK/.TMP */
        if (!suffix_ok) return false;
        out_index = static_cast<uint16_t>(value);
        return true;
    }

    bool rebuild_last_from_directory(uint16_t &last)
    {
        last = 0U;
        last_result_ = f_mkdir("/SESSIONS");
        if (last_result_ != FR_OK && last_result_ != FR_EXIST) return false;
        DIR dir{};
        last_result_ = f_opendir(&dir, "/SESSIONS");
        if (last_result_ == FR_NO_PATH) return true; /* empty card: index 1 next */
        if (last_result_ != FR_OK) return false;
        FILINFO info{};
        for (;;) {
            last_result_ = f_readdir(&dir, &info);
            if (last_result_ != FR_OK || info.fname[0] == '\0') break;
            uint16_t index = 0U;
            if (parse_session_index(info.fname, index) && index > last) {
                last = index;
            }
        }
        const FRESULT read_result = last_result_;
        f_closedir(&dir);
        if (read_result != FR_OK) {
            last_result_ = read_result;
            return false;
        }
        return true;
    }

    bool ensure_fs()
    {
        if (USERFatFs.fs_type == 0U) {
            last_result_ = f_mount(&USERFatFs, USERPath, 1U);
            if (last_result_ != FR_OK) return false;
        }
        last_result_ = f_mkdir("/SESSIONS");
        if (last_result_ != FR_OK && last_result_ != FR_EXIST) return false;
        return true;
    }

    bool load_cache()
    {
        cache_ready_ = false;
        if (!ensure_fs()) {
            return false;
        }

        FIL file{};
        uint8_t marker[8] = {0};
        UINT read = 0U;
        last_result_ = f_open(&file, kMarkerPath, FA_READ);
        if (last_result_ == FR_OK) {
            last_result_ = f_read(&file, marker, sizeof(marker), &read);
            f_close(&file);
        } else if (last_result_ == FR_NO_FILE) {
            last_result_ = FR_OK;
        } else {
            return false;
        }

        uint16_t last = 0U;
        const bool marker_valid = (last_result_ == FR_OK) && (read == sizeof(marker)) &&
                (marker[0] == static_cast<uint8_t>(kMarkerMagic & 0xFFU)) &&
                (marker[1] == static_cast<uint8_t>((kMarkerMagic >> 8) & 0xFFU)) &&
                (marker[2] == static_cast<uint8_t>((kMarkerMagic >> 16) & 0xFFU)) &&
                (marker[3] == static_cast<uint8_t>((kMarkerMagic >> 24) & 0xFFU)) &&
                (marker[4] <= 0x0FU) && (marker[5] <= 0x27U); /* index <= 9999 */
        if (marker_valid) {
            last = static_cast<uint16_t>(marker[4] | (marker[5] << 8));
        } else {
            /* No usable marker: rebuild from a single enumeration of
             * /SESSIONS. One directory pass, never a per-index f_stat storm —
             * an O(index*8) scan here would block boot for minutes on a card
             * that already holds sessions. */
            if (!rebuild_last_from_directory(last)) {
                return false;
            }
            if (!persist_marker(last)) return false;
        }
        cached_last_ = last;
        cache_ready_ = true;
        return true;
    }

    bool persist_marker(uint16_t index)
    {
        FIL file{};
        uint8_t marker[8] = {0};
        marker[0] = static_cast<uint8_t>(kMarkerMagic & 0xFFU);
        marker[1] = static_cast<uint8_t>((kMarkerMagic >> 8) & 0xFFU);
        marker[2] = static_cast<uint8_t>((kMarkerMagic >> 16) & 0xFFU);
        marker[3] = static_cast<uint8_t>((kMarkerMagic >> 24) & 0xFFU);
        marker[4] = static_cast<uint8_t>(index & 0xFFU);
        marker[5] = static_cast<uint8_t>((index >> 8) & 0xFFU);
        UINT written = 0U;
        last_result_ = f_open(&file, kMarkerPath, FA_CREATE_ALWAYS | FA_WRITE);
        if (last_result_ != FR_OK) return false;
        last_result_ = f_write(&file, marker, sizeof(marker), &written);
        const FRESULT write_result = last_result_;
        const FRESULT close_result = f_close(&file);
        if (write_result != FR_OK || close_result != FR_OK || written != sizeof(marker)) {
            last_result_ = (write_result != FR_OK) ? write_result : close_result;
            return false;
        }
        return true;
    }

    static void append_index(char *path, size_t &cursor, uint16_t index)
    {
        path[cursor++] = static_cast<char>('0' + ((index / 1000U) % 10U));
        path[cursor++] = static_cast<char>('0' + ((index / 100U) % 10U));
        path[cursor++] = static_cast<char>('0' + ((index / 10U) % 10U));
        path[cursor++] = static_cast<char>('0' + (index % 10U));
    }

    static void make_path(char *path, size_t path_size, const char *prefix,
            uint16_t index, const char *suffix)
    {
        if (path == nullptr || path_size == 0U) return;
        size_t cursor = 0U;
        for (size_t i = 0U; prefix[i] != '\0' && cursor + 1U < path_size; ++i)
            path[cursor++] = prefix[i];
        if (cursor + 4U >= path_size) { path[0] = '\0'; return; }
        append_index(path, cursor, index);
        for (size_t i = 0U; suffix[i] != '\0' && cursor + 1U < path_size; ++i)
            path[cursor++] = suffix[i];
        path[cursor] = '\0';
    }

    bool path_exists(const char *path, bool &exists)
    {
        FILINFO info{};
        last_result_ = f_stat(path, &info);
        if (last_result_ == FR_OK) { exists = true; return true; }
        if (last_result_ == FR_NO_FILE || last_result_ == FR_NO_PATH) {
            exists = false;
            last_result_ = FR_OK;
            return true;
        }
        return false;
    }

    bool index_occupied(uint16_t index, bool &occupied)
    {
        occupied = false;
        char path[32]{};
        static const char *const binary_suffixes[] = {
            "M.BIN", "N1.BIN", "N2.BIN", "N3.BIN", "N4.BIN"
        };
        static const char *const legacy_csv_suffixes[] = { ".TMP", ".CSV", ".OK" };
        for (const char *suffix : binary_suffixes) {
            make_path(path, sizeof(path), "/SESSIONS/R", index, suffix);
            bool exists = false;
            if (!path_exists(path, exists)) return false;
            if (exists) { occupied = true; return true; }
        }
        for (const char *suffix : legacy_csv_suffixes) {
            make_path(path, sizeof(path), "/SESSIONS/TRN", index, suffix);
            bool exists = false;
            if (!path_exists(path, exists)) return false;
            if (exists) { occupied = true; return true; }
        }
        return true;
    }

    static constexpr const char *kMarkerPath = "/SESSIONS/RUNIDX.BIN";

    FRESULT last_result_ = FR_OK;
    bool cache_ready_ = false;
    uint16_t cached_last_ = 0U;
};

} // namespace exo

#endif

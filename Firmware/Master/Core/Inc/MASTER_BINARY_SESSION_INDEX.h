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

    bool allocate(uint16_t &out_index)
    {
        out_index = 0U;
        if (USERFatFs.fs_type == 0U) {
            last_result_ = f_mount(&USERFatFs, USERPath, 1U);
            if (last_result_ != FR_OK) return false;
        }

        last_result_ = f_mkdir("/SESSIONS");
        if (last_result_ != FR_OK && last_result_ != FR_EXIST) return false;

        for (uint16_t index = 1U; index <= kMaxFileIndex; ++index) {
            bool occupied = false;
            if (!index_occupied(index, occupied)) return false;
            if (!occupied) {
                out_index = index;
                last_result_ = FR_OK;
                return true;
            }
        }
        last_result_ = FR_EXIST;
        return false;
    }

    FRESULT last_result() const { return last_result_; }

private:
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

    FRESULT last_result_ = FR_OK;
};

} // namespace exo

#endif

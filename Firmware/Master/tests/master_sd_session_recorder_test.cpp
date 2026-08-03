#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern "C" {
#include "ff.h"
}

FATFS USERFatFs{};
char USERPath[4]{};

#include "MASTER_SD_SESSION_RECORDER.h"

namespace {

struct FakeRecorderFs {
    FSIZE_t cursor = 0U;
    bool fail_bno_write = false;
    bool fail_icm_write = false;
    unsigned bno_write_attempts = 0U;
    unsigned icm_write_attempts = 0U;
    exo::Bno85Sample bno[16]{};
    uint32_t bno_count = 0U;
    exo::Icm45686Sample icm[64]{};
    uint32_t icm_count = 0U;
} g_fs;

constexpr uint32_t kBnoStart = sizeof(exo::SessionHeader);
constexpr uint32_t kIcmStart = kBnoStart +
        360000U * static_cast<uint32_t>(sizeof(exo::Bno85Sample));

void reset_fs()
{
    g_fs = FakeRecorderFs{};
    memset(&USERFatFs, 0, sizeof(USERFatFs));
}

} // namespace

extern "C" FRESULT f_mount(FATFS *fs, const TCHAR *, BYTE)
{
    fs->fs_type = 1U;
    return FR_OK;
}

extern "C" FRESULT f_mkdir(const TCHAR *) { return FR_EXIST; }

extern "C" FRESULT f_open(FIL *, const TCHAR *, BYTE)
{
    g_fs.cursor = 0U;
    return FR_OK;
}

extern "C" FRESULT f_lseek(FIL *, FSIZE_t offset)
{
    g_fs.cursor = offset;
    return FR_OK;
}

extern "C" FRESULT f_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    *written = 0U;
    if (g_fs.cursor >= kIcmStart) {
        ++g_fs.icm_write_attempts;
        if (g_fs.fail_icm_write) {
            return FR_DISK_ERR;
        }
        const uint32_t count = bytes / static_cast<uint32_t>(sizeof(exo::Icm45686Sample));
        if (g_fs.icm_count + count > 64U) {
            return FR_DISK_ERR;
        }
        memcpy(&g_fs.icm[g_fs.icm_count], data, bytes);
        g_fs.icm_count += count;
    } else if (g_fs.cursor >= kBnoStart) {
        ++g_fs.bno_write_attempts;
        if (g_fs.fail_bno_write) {
            return FR_DISK_ERR;
        }
        const uint32_t count = bytes / static_cast<uint32_t>(sizeof(exo::Bno85Sample));
        if (g_fs.bno_count + count > 16U) {
            return FR_DISK_ERR;
        }
        memcpy(&g_fs.bno[g_fs.bno_count], data, bytes);
        g_fs.bno_count += count;
    }
    g_fs.cursor += bytes;
    *written = bytes;
    return FR_OK;
}

extern "C" FRESULT f_read(FIL *, void *data, UINT bytes, UINT *received)
{
    if (g_fs.cursor >= kIcmStart) {
        const uint32_t offset = static_cast<uint32_t>(g_fs.cursor - kIcmStart);
        memcpy(data, reinterpret_cast<const uint8_t *>(g_fs.icm) + offset, bytes);
    } else if (g_fs.cursor >= kBnoStart) {
        const uint32_t offset = static_cast<uint32_t>(g_fs.cursor - kBnoStart);
        memcpy(data, reinterpret_cast<const uint8_t *>(g_fs.bno) + offset, bytes);
    } else {
        memset(data, 0, bytes);
    }
    g_fs.cursor += bytes;
    *received = bytes;
    return FR_OK;
}

extern "C" FRESULT f_sync(FIL *) { return FR_OK; }
extern "C" FRESULT f_close(FIL *) { return FR_OK; }

int main()
{
    reset_fs();
    exo::MasterSdSessionRecorder recorder;
    if (!recorder.start(0U, 77U, 0U, 1000U)) {
        return 1;
    }
    exo::Icm45686Sample icm{};
    for (uint32_t index = 0U; index < 33U; ++index) {
        icm.sequence = 0xFFFFFFFFU;
        if (!recorder.append_icm45686(icm)) {
            return 2;
        }
    }
    if (!recorder.finalize(1000U) || g_fs.icm_count != 33U ||
            g_fs.icm[0].sequence != 0U || g_fs.icm[31].sequence != 31U ||
            g_fs.icm[32].sequence != 32U) {
        return 3;
    }

    reset_fs();
    exo::MasterSdSessionRecorder failed_icm;
    if (!failed_icm.start(0U, 78U, 0U, 1000U)) {
        return 4;
    }
    g_fs.fail_icm_write = true;
    for (uint32_t index = 0U; index < 31U; ++index) {
        if (!failed_icm.append_icm45686(icm)) {
            return 5;
        }
    }
    if (failed_icm.append_icm45686(icm) || g_fs.icm_write_attempts != 1U ||
            failed_icm.append_icm45686(icm) || g_fs.icm_write_attempts != 1U) {
        return 6;
    }

    reset_fs();
    exo::MasterSdSessionRecorder failed_bno;
    if (!failed_bno.start(0U, 79U, 0U, 1000U)) {
        return 7;
    }
    exo::Bno85Sample bno{};
    g_fs.fail_bno_write = true;
    for (uint32_t index = 0U; index < 7U; ++index) {
        if (!failed_bno.append_bno85(bno)) {
            return 8;
        }
    }
    if (failed_bno.append_bno85(bno) || g_fs.bno_write_attempts != 1U ||
            failed_bno.append_bno85(bno) || g_fs.bno_write_attempts != 1U) {
        return 9;
    }

    return 0;
}

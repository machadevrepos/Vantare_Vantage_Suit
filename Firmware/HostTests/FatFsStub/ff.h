#ifndef EXO_HOST_FF_H_
#define EXO_HOST_FF_H_

#include <stdint.h>

typedef char TCHAR;
typedef uint8_t BYTE;
typedef unsigned int UINT;
typedef uint32_t DWORD;
typedef uint32_t FSIZE_t;

typedef struct { uint32_t reserved; } FIL;
typedef struct { uint32_t fsize; } FILINFO;
typedef struct { BYTE fs_type; } FATFS;

typedef enum {
    FR_OK = 0,
    FR_DISK_ERR,
    FR_INT_ERR,
    FR_NOT_READY,
    FR_NO_FILE,
    FR_NO_PATH,
    FR_INVALID_NAME,
    FR_DENIED,
    FR_EXIST,
    FR_INVALID_OBJECT,
    FR_WRITE_PROTECTED,
    FR_INVALID_DRIVE,
    FR_NOT_ENABLED,
    FR_NO_FILESYSTEM,
    FR_MKFS_ABORTED,
    FR_TIMEOUT,
    FR_LOCKED,
    FR_NOT_ENOUGH_CORE,
    FR_TOO_MANY_OPEN_FILES,
    FR_INVALID_PARAMETER
} FRESULT;

#define FA_READ          0x01U
#define FA_WRITE         0x02U
#define FA_OPEN_EXISTING 0x00U
#define FA_CREATE_NEW    0x04U
#define FA_CREATE_ALWAYS 0x08U
#define FA_OPEN_ALWAYS   0x10U
#define FA_OPEN_APPEND   0x30U

static inline FRESULT f_mount(FATFS*, const TCHAR*, BYTE) { return FR_OK; }
static inline FRESULT f_mkdir(const TCHAR*) { return FR_OK; }
static inline FRESULT f_stat(const TCHAR*, FILINFO*) { return FR_NO_FILE; }
static inline FRESULT f_open(FIL*, const TCHAR*, BYTE) { return FR_OK; }
static inline FRESULT f_write(FIL*, const void*, UINT bytes, UINT *written)
{
    if (written != 0) *written = bytes;
    return FR_OK;
}
static inline FRESULT f_read(FIL*, void*, UINT bytes, UINT *read)
{
    if (read != 0) *read = bytes;
    return FR_OK;
}
static inline FRESULT f_lseek(FIL*, FSIZE_t) { return FR_OK; }
static inline FSIZE_t f_size(FIL*) { return 0U; }
static inline FRESULT f_sync(FIL*) { return FR_OK; }
static inline FRESULT f_close(FIL*) { return FR_OK; }
static inline FRESULT f_rename(const TCHAR*, const TCHAR*) { return FR_OK; }
static inline FRESULT f_unlink(const TCHAR*) { return FR_OK; }

#endif

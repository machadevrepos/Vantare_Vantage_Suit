#ifndef SD_LOGGING_HPP
#define SD_LOGGING_HPP

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
extern "C"
{
#include <FatFs/ff.h>
#include <FatFs/ff_gen_drv.h>
#include <FatFs/sd_diskio.h>
}

class SDLogger
{
public:
    using TimestampProvider = void (*)(std::string &);
    TimestampProvider timestampProvider;

    void setTimestampProvider(TimestampProvider provider)
    {
        timestampProvider = provider;
    }

   static constexpr const char *LOG_ROOT_DIR = "/LOGS/data_logs";
   static constexpr const char *DATA_SUFFIX = "_data.bin";
   static constexpr const char *INDEX_SUFFIX = "_index.bin";
   static constexpr const char *PUBLISHED_SUFFIX = "_published.flag";
   static constexpr const char *STATE_FILE = "/LOGS/data_log_state.bin";
   static constexpr uint32_t MAX_TOPIC_LEN = 32;
   static constexpr uint32_t TIMESTAMP_LEN = 20;
   static constexpr uint32_t DATE_LEN = 8;
   static constexpr uint32_t INDEX_ENTRY_SIZE = 64;
   static constexpr uint32_t STATE_SIZE = 44;
   static constexpr uint32_t MAX_PATH = 128;

    struct IndexEntry
    {
        std::string timestamp; // "YYYY-MM-DDTHH:MM:SS"
        std::string topic;
        uint32_t data_offset;
        uint32_t payload_len;
        uint8_t published;
        uint8_t reserved[3];
        IndexEntry() : timestamp(TIMESTAMP_LEN, '\0'), topic(MAX_TOPIC_LEN, '\0'), data_offset(0), payload_len(0), published(0) { reserved[0] = reserved[1] = reserved[2] = 0; }
    };

    struct State
    {
        uint32_t version;
        std::string current_log_date;
        std::string last_published_date;
        std::string last_published_timestamp;
        uint32_t last_published_index_offset;
        State() : version(0), current_log_date(DATE_LEN, '\0'), last_published_date(DATE_LEN, '\0'), last_published_timestamp(TIMESTAMP_LEN, '\0'), last_published_index_offset(0) {}
    };

    struct RecordInfo
    {
        std::string date;
        uint32_t index_offset;
        uint32_t data_offset;
        uint32_t payload_len;
        std::string topic;
        std::string timestamp;
        uint8_t published;
        RecordInfo() : date(DATE_LEN, '\0'), index_offset(0), data_offset(0), payload_len(0), topic(MAX_TOPIC_LEN, '\0'), timestamp(TIMESTAMP_LEN, '\0'), published(0) {}
    };

    bool mount()
    {
        return FATFS_MountSD() == FR_OK;
    }
    bool unmount()
    {
        extern char SDPath[4];
        return f_mount(NULL, SDPath, 1) == FR_OK;
    }

    void getCurrentTimestamp(std::string &timestamp)
    {
        if (timestampProvider)
        {
            timestampProvider(timestamp);
        }
        else
        {
            timestamp = "0000-00-00T00:00:00";
        }
    }
    bool extractDate(const std::string &timestamp, std::string &date)
    {
        date.clear();
        for (size_t i = 0; i < timestamp.size() && date.size() < DATE_LEN; ++i)
        {
            if (timestamp[i] != '-' && timestamp[i] != 'T')
            {
                date += timestamp[i];
            }
        }
        return (date.size() == DATE_LEN);
    }
    bool isValidDate(const std::string &date)
    {
        for (char c : date)
        {
            if (c < '0' || c > '9')
                return false;
        }
        return true;
    }
    void copyField(std::string &dst, const std::string &src, size_t maxLen)
    {
        dst = src.substr(0, maxLen);
    }
    bool writeFull(FIL *fd, const void *data, uint32_t len)
    {
        if (!fd || !data || len == 0)
            return false;
        UINT bw;
        return (f_write(fd, data, len, &bw) == FR_OK && bw == len);
    }
    bool readFull(FIL *fd, void *data, uint32_t len)
    {
        if (!fd || !data || len == 0)
            return false;
        UINT br;
        return (f_read(fd, data, len, &br) == FR_OK && br == len);
    }

    bool sd_save_log(const char *topic, const void *data, uint32_t len, bool published, RecordInfo *out_info)
    {
        std::string date;
        std::string timestamp;
        getCurrentTimestamp(timestamp);
        extractDate(timestamp, date);
        uint32_t data_offset = 0;
        if (!sd_store_payload(date, data, len, &data_offset))
            return false;
        IndexEntry entry;
        strncpy(entry.timestamp.data(), timestamp.c_str(), TIMESTAMP_LEN);
        strncpy(entry.topic.data(), topic, MAX_TOPIC_LEN);
        entry.data_offset = data_offset;
        entry.payload_len = len;
        entry.published = published ? 1 : 0;
        uint32_t index_offset = 0;
        if (!sd_append_index_entry(date, &entry, &index_offset))
            return false;
        if (out_info)
        {
            *out_info = RecordInfo(); // value-initialize instead of memset
            strncpy(out_info->date.data(), date.c_str(), DATE_LEN);
            out_info->index_offset = index_offset;
            out_info->data_offset = data_offset;
            out_info->payload_len = len;
            strncpy(out_info->topic.data(), topic, MAX_TOPIC_LEN);
            strncpy(out_info->timestamp.data(), timestamp.c_str(), TIMESTAMP_LEN);
            out_info->published = entry.published;
        }
        return true;
    }

    bool sd_store_payload(const std::string &log_date, const void *data, uint32_t len, uint32_t *out_offset)
    {
        std::string path = std::string(LOG_ROOT_DIR) + "/data_log_" + log_date + DATA_SUFFIX;
        FIL file;
        if (f_open(&file, path.c_str(), FA_OPEN_APPEND | FA_WRITE) != FR_OK)
            return false;
        *out_offset = f_size(&file);
        UINT bw;
        uint32_t payload_len = len;
        if (f_write(&file, &payload_len, sizeof(payload_len), &bw) != FR_OK || bw != sizeof(payload_len))
        {
            f_close(&file);
            return false;
        }
        if (f_write(&file, data, len, &bw) != FR_OK || bw != len)
        {
            f_close(&file);
            return false;
        }
        f_close(&file);
        return true;
    }

    bool sd_append_index_entry(const std::string &log_date, const IndexEntry *entry, uint32_t *out_offset)
    {
        std::string path = std::string(LOG_ROOT_DIR) + "/data_log_" + log_date + INDEX_SUFFIX;
        FIL file;
        if (f_open(&file, path.c_str(), FA_OPEN_APPEND | FA_WRITE) != FR_OK)
            return false;
        *out_offset = f_size(&file);
        UINT bw;
        if (f_write(&file, entry, sizeof(IndexEntry), &bw) != FR_OK || bw != sizeof(IndexEntry))
        {
            f_close(&file);
            return false;
        }
        f_close(&file);
        return true;
    }

    bool sd_mark_date_published(const std::string &date)
    {
        std::string path = std::string(LOG_ROOT_DIR) + "/data_log_" + date + PUBLISHED_SUFFIX;
        FIL file;
        if (f_open(&file, path.c_str(), FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
            return false;
        f_close(&file);
        return true;
    }

    bool sd_clear_date_published_flag(const std::string &date)
    {
        std::string path = std::string(LOG_ROOT_DIR) + "/data_log_" + date + PUBLISHED_SUFFIX;
        return (f_unlink(path.c_str()) == FR_OK);
    }

    bool sd_build_log_path(const std::string &date, const std::string &suffix, std::string &out_path)
    {
        if (date.empty() || suffix.empty())
            return false;
        out_path = std::string(LOG_ROOT_DIR) + "/data_log_" + date + suffix;
        return true;
    }

    bool sd_read_state(State *state)
    {
        FIL file;
        if (f_open(&file, STATE_FILE, FA_READ) != FR_OK)
            return false;
        UINT br;
        if (f_read(&file, state, sizeof(State), &br) != FR_OK || br != sizeof(State))
        {
            f_close(&file);
            return false;
        }
        f_close(&file);
        return true;
    }

    bool sd_write_state(const State *state)
    {
        FIL file;
        if (f_open(&file, STATE_FILE, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
            return false;
        UINT bw;
        if (f_write(&file, state, sizeof(State), &bw) != FR_OK || bw != sizeof(State))
        {
            f_close(&file);
            return false;
        }
        f_close(&file);
        return true;
    }

    bool sd_load_state_file(State *state)
    {
        FIL file;
        UINT br;
        // Try to read the current state file
        if (f_open(&file, STATE_FILE, FA_READ) == FR_OK)
        {
            if (f_read(&file, state, sizeof(State), &br) == FR_OK && br == sizeof(State))
            {
                f_close(&file);
                return true;
            }
            f_close(&file);
        }
        // If failed, try legacy file (example: /sdcard0/data_log_data.bin)
        FIL legacy_file;
        if (f_open(&legacy_file, "/sdcard0/data_log_data.bin", FA_READ) == FR_OK)
        {
            *state = State(); // value-initialize instead of memset
            // Example: read first 8 bytes as last_published_date
            if (f_read(&legacy_file, state->last_published_date.data(), DATE_LEN, &br) == FR_OK && br == DATE_LEN)
            {
                // Optionally read more legacy fields here
                f_close(&legacy_file);
                return true;
            }
            f_close(&legacy_file);
        }
        return false;
    }

private:
    static SDLogger* instance;
    static bool instanceCreated;

    // Private constructor to prevent direct instantiation
    SDLogger() = default;
    SDLogger(const SDLogger&) = delete;
    SDLogger& operator=(const SDLogger&) = delete;

public:
    static SDLogger* getInstance() {
        static SDLogger singleton;
        if (!instanceCreated) {
            instance = &singleton;
            instanceCreated = true;
        }
        return instance;
    }
};

#endif // SD_LOGGING_HPP

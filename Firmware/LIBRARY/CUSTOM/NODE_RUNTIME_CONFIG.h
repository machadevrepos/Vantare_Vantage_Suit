#ifndef NODE_RUNTIME_CONFIG_H_
#define NODE_RUNTIME_CONFIG_H_

#include <stdint.h>
#include <string.h>

namespace exo::node_runtime_config {

static constexpr uint8_t kNodeIdMin = 1U;
static constexpr uint8_t kNodeIdMax = 5U;
static constexpr uint32_t kDefaultFlashTotalSize = 2U * 1024U * 1024U;
static constexpr uint32_t kSettingsSectorSize = 4096U;
static constexpr uint32_t kSettingsMagic = 0x4E535447UL; /* 'NSTG' */
static constexpr uint16_t kSettingsVersion = 1U;

using ReadFn = bool (*)(uint32_t address, void *data, uint32_t size);
using WriteFn = bool (*)(uint32_t address, const void *data, uint32_t size);
using EraseFn = bool (*)(uint32_t address, uint32_t size);

#pragma pack(push, 1)
struct NodePersistentSettings {
    uint32_t magic;
    uint16_t version;
    uint8_t node_id;
    uint8_t flags;
    uint32_t reserved;
    uint32_t crc32;
};
#pragma pack(pop)

inline ReadFn &read_hook() {
    static ReadFn fn = nullptr;
    return fn;
}

inline WriteFn &write_hook() {
    static WriteFn fn = nullptr;
    return fn;
}

inline EraseFn &erase_hook() {
    static EraseFn fn = nullptr;
    return fn;
}

inline uint32_t &flash_capacity_bytes() {
    static uint32_t capacity = kDefaultFlashTotalSize;
    return capacity;
}

inline uint32_t settings_sector_base() {
    return flash_capacity_bytes() - kSettingsSectorSize;
}

inline bool set_flash_capacity(uint32_t capacity) {
    if (capacity < (2U * kSettingsSectorSize) || (capacity % kSettingsSectorSize) != 0U) {
        return false;
    }
    flash_capacity_bytes() = capacity;
    return true;
}

inline uint32_t crc32_ieee(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]);
        for (uint8_t b = 0U; b < 8U; ++b) {
            const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1U)));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

inline bool storage_ready() {
    return read_hook() != nullptr && write_hook() != nullptr && erase_hook() != nullptr;
}

inline void set_storage_hooks(ReadFn read_fn, WriteFn write_fn, EraseFn erase_fn) {
    read_hook() = read_fn;
    write_hook() = write_fn;
    erase_hook() = erase_fn;
}

inline bool is_valid_node_id(uint8_t node_id) {
    return (node_id >= kNodeIdMin) && (node_id <= kNodeIdMax);
}

inline bool load_settings(NodePersistentSettings &out) {
    if (!storage_ready()) {
        return false;
    }
    if (!read_hook()(settings_sector_base(), &out, static_cast<uint32_t>(sizeof(out)))) {
        return false;
    }
    if (out.magic != kSettingsMagic || out.version != kSettingsVersion || !is_valid_node_id(out.node_id)) {
        return false;
    }
    const uint32_t calc = crc32_ieee(reinterpret_cast<const uint8_t *>(&out), static_cast<uint32_t>(sizeof(out) - sizeof(out.crc32)));
    return calc == out.crc32;
}

inline bool store_node_id(uint8_t node_id) {
    if (!is_valid_node_id(node_id) || !storage_ready()) {
        return false;
    }
    NodePersistentSettings settings{};
    settings.magic = kSettingsMagic;
    settings.version = kSettingsVersion;
    settings.node_id = node_id;
    settings.flags = 0U;
    settings.reserved = 0U;
    settings.crc32 = crc32_ieee(reinterpret_cast<const uint8_t *>(&settings),
                                static_cast<uint32_t>(sizeof(settings) - sizeof(settings.crc32)));
    if (!erase_hook()(settings_sector_base(), kSettingsSectorSize)) {
        return false;
    }
    if (!write_hook()(settings_sector_base(), &settings, static_cast<uint32_t>(sizeof(settings)))) {
        return false;
    }
    NodePersistentSettings verify{};
    if (!read_hook()(settings_sector_base(), &verify, static_cast<uint32_t>(sizeof(verify)))) {
        return false;
    }
    return (memcmp(&settings, &verify, sizeof(settings)) == 0);
}

inline uint8_t load_node_id(uint8_t default_id) {
    const uint8_t fallback = is_valid_node_id(default_id) ? default_id : kNodeIdMin;
    NodePersistentSettings settings{};
    if (load_settings(settings)) {
        return settings.node_id;
    }
    (void)store_node_id(fallback);
    return fallback;
}

} // namespace exo::node_runtime_config

#endif

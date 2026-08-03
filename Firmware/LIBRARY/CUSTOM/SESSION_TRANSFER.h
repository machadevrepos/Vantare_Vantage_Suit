#ifndef SESSION_TRANSFER_H_
#define SESSION_TRANSFER_H_

#include <string.h>

#include "NODE_RECORDER.h"

namespace exo {

class SessionUploadReader {
public:
    SessionUploadReader(SessionFlash &flash, const SessionHeader &header, uint32_t header_address,
                        uint32_t bno85_payload_address, uint32_t icm45686_payload_address)
        : flash_(flash),
          header_(header),
          header_address_(header_address),
          bno85_payload_address_(bno85_payload_address),
          icm45686_payload_address_(icm45686_payload_address) {}

    uint32_t total_size() const {
        return sizeof(SessionHeader) + header_.bno85_payload_size + header_.icm45686_payload_size;
    }

    bool read(uint32_t offset, void *buffer, uint32_t size) {
        if ((offset + size) > total_size()) {
            return false;
        }
        uint8_t *out = static_cast<uint8_t *>(buffer);
        uint32_t remaining = size;
        uint32_t logical = offset;

        while (remaining > 0U) {
            if (logical < sizeof(SessionHeader)) {
                const uint32_t span = min_u32(remaining, sizeof(SessionHeader) - logical);
                if (!flash_.read(header_address_ + logical, out, span)) {
                    return false;
                }
                logical += span;
                out += span;
                remaining -= span;
                continue;
            }

            const uint32_t bno_logical_start = sizeof(SessionHeader);
            const uint32_t icm_logical_start = bno_logical_start + header_.bno85_payload_size;
            if (logical < icm_logical_start) {
                const uint32_t offset_in_bno = logical - bno_logical_start;
                const uint32_t span = min_u32(remaining, header_.bno85_payload_size - offset_in_bno);
                if (!flash_.read(bno85_payload_address_ + offset_in_bno, out, span)) {
                    return false;
                }
                logical += span;
                out += span;
                remaining -= span;
                continue;
            }

            const uint32_t offset_in_icm = logical - icm_logical_start;
            const uint32_t span = min_u32(remaining, header_.icm45686_payload_size - offset_in_icm);
            if (!flash_.read(icm45686_payload_address_ + offset_in_icm, out, span)) {
                return false;
            }
            logical += span;
            out += span;
            remaining -= span;
        }
        return true;
    }

private:
    static uint32_t min_u32(uint32_t a, uint32_t b) {
        return a < b ? a : b;
    }

    SessionFlash &flash_;
    const SessionHeader &header_;
    uint32_t header_address_;
    uint32_t bno85_payload_address_;
    uint32_t icm45686_payload_address_;
};

class HubSessionAssembler {
public:
    bool begin(const SessionHeader &header, uint8_t *payload_buffer, uint32_t payload_capacity) {
        const uint32_t payload_size = header.bno85_payload_size + header.icm45686_payload_size;
        if (header.magic != kSessionMagic ||
            header.version != kSessionFormatVersion ||
            header.completion_flag != kSessionComplete ||
            header.header_crc32 != session_header_crc(header) ||
            payload_buffer == nullptr ||
            payload_capacity < payload_size) {
            return false;
        }
        header_ = header;
        payload_ = payload_buffer;
        payload_capacity_ = payload_capacity;
        next_offset_ = 0U;
        return true;
    }

    bool append_payload_chunk(uint32_t payload_offset, const uint8_t *data, uint32_t size) {
        const uint32_t payload_size = header_.bno85_payload_size + header_.icm45686_payload_size;
        if (data == nullptr ||
            payload_offset != next_offset_ ||
            (payload_offset + size) > payload_size ||
            (payload_offset + size) > payload_capacity_) {
            return false;
        }
        memcpy(payload_ + payload_offset, data, size);
        next_offset_ += size;
        return true;
    }

    uint32_t next_offset() const { return next_offset_; }

    bool complete() const {
        const uint32_t payload_size = header_.bno85_payload_size + header_.icm45686_payload_size;
        return next_offset_ == payload_size && crc32(payload_, payload_size) == header_.payload_crc32;
    }

    const SessionHeader &header() const { return header_; }

private:
    SessionHeader header_{};
    uint8_t *payload_ = nullptr;
    uint32_t payload_capacity_ = 0U;
    uint32_t next_offset_ = 0U;
};

} // namespace exo

#endif

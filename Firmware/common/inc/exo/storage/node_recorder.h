#ifndef NODE_RECORDER_H_
#define NODE_RECORDER_H_

#include <string.h>

#include <exo/types/recording_types.h>

namespace exo {

class SessionFlash {
public:
    virtual ~SessionFlash() = default;
    virtual bool erase_region(uint32_t address, uint32_t size) = 0;
    /* Single 4 KB sector erase, the chunk unit of the background eraser. */
    virtual bool erase_4k(uint32_t address) { return erase_region(address, 4096U); }
    virtual bool write(uint32_t address, const void *data, uint32_t size) = 0;
    /* Fast path for regions the caller guarantees pre-erased: skips the
     * driver's 4 KB sector read-modify-write. Default falls back to write(). */
    virtual bool write_pre_erased(uint32_t address, const void *data, uint32_t size) { return write(address, data, size); }
    virtual bool read(uint32_t address, void *data, uint32_t size) = 0;
};

class NodeRecorder {
public:
    /* Background region eraser. A whole-region erase takes seconds (a 10-min
     * layout is ~1400 sectors, ~45 ms each), far too long to run inside a BLE
     * command handler or the capture window, so the erase advances in bounded
     * chunks from the node superloop. Completion is remembered so a follow-up
     * session after a successful upload skips the erase entirely. */
    enum class EraseStep : uint8_t { Idle, InProgress, Complete, Failed };
    static constexpr uint32_t kSectorSize = 4096U;

    NodeRecorder(SessionFlash &flash, uint32_t base_address, uint32_t capacity, uint32_t bno85_region_size)
        : flash_(flash), base_address_(base_address), capacity_(capacity), bno85_region_size_(bno85_region_size),
          header_address_(base_address) {
        reset_runtime();
    }

    /* Crash-consistent session header: the header lives in the reserved
     * recovery sector instead of sector 0, so the finalize rewrite can never
     * erase already-stored payload. Two 88 B slots are used: slot A (offset 0)
     * carries the in-progress header from session start; slot B (offset 256,
     * still 0xFF after the region erase) receives the finalized header as a
     * single page program — no erase window. Boot salvage adopts a valid
     * slot B as ReadyForUpload, so a crash after finalize (including one
     * mid-upload) recovers instead of orphaning the session. */
    static constexpr uint32_t kHeaderSlotStride = 256U;
    uint32_t finalized_header_address() const { return header_address_ + kHeaderSlotStride; }

    void set_header_address(uint32_t address) {
        header_address_ = address;
    }
    uint32_t header_address() const { return header_address_; }

    /* Adopt a finalized session found in recovery slot B at boot. Returns
     * true when the node boots straight into ReadyForUpload. */
    bool recover_after_boot() {
        SessionHeader candidate { };
        if (!flash_.read(finalized_header_address(), &candidate, sizeof(candidate))) {
            return false;
        }
        if (candidate.magic != kSessionMagic ||
                candidate.version != kSessionFormatVersion ||
                candidate.completion_flag != kSessionComplete ||
                candidate.header_crc32 != session_header_crc(candidate) ||
                candidate.bno85_payload_size == 0U) {
            return false;
        }
        header_ = candidate;
        bno85_cursor_ = bno85_payload_address();
        icm45686_cursor_ = icm45686_payload_address() + header_.icm45686_payload_size;
        state_ = RecorderState::ReadyForUpload;
        region_erased_ = false;
        return true;
    }

    RecorderState state() const { return state_; }
    const SessionHeader &header() const { return header_; }

    bool configure_layout(uint32_t capacity, uint32_t bno85_region_size) {
        if ((state_ != RecorderState::Idle && state_ != RecorderState::ReadyForUpload) ||
            capacity == 0U || bno85_region_size > capacity) {
            return false;
        }
        const bool layout_changed = capacity_ != capacity || bno85_region_size_ != bno85_region_size;
        const bool was_erased = region_erased_;
        capacity_ = capacity;
        bno85_region_size_ = bno85_region_size;
        reset_runtime();
        /* An unchanged layout keeps the pre-erased state, so back-to-back
         * sessions of the same duration do not re-erase the region. */
        region_erased_ = layout_changed ? false : was_erased;
        return true;
    }

    void set_capture_metadata(uint16_t bno_rate_hz, uint16_t icm_rate_hz,
                              uint32_t bno_attempted, uint32_t icm_attempted,
                              uint32_t bno_captured, uint32_t icm_captured,
                              uint32_t bno_dropped, uint32_t icm_dropped,
                              uint32_t loss_flags) {
        header_.bno85_target_rate_hz = bno_rate_hz;
        header_.icm45686_target_rate_hz = icm_rate_hz;
        header_.bno85_attempted_count = bno_attempted;
        header_.icm45686_attempted_count = icm_attempted;
        header_.bno85_captured_count = bno_captured;
        header_.icm45686_captured_count = icm_captured;
        header_.bno85_dropped_count = bno_dropped;
        header_.icm45686_dropped_count = icm_dropped;
        header_.loss_flags = loss_flags;
    }

    /* Region-erase machine. begin_region_erase() completes immediately when
     * the region is already known-erased; otherwise service_region_erase()
     * erases up to max_sectors 4 KB sectors per call. Never erases under an
     * active upload of the same region. */
    EraseStep begin_region_erase() {
        if (erase_active_) {
            return EraseStep::InProgress;
        }
        if (region_erased_) {
            return EraseStep::Complete;
        }
        erase_cursor_ = 0U;
        erase_fail_streak_ = 0U;
        erase_active_ = true;
        return EraseStep::InProgress;
    }

    EraseStep service_region_erase(uint8_t max_sectors) {
        if (!erase_active_) {
            return region_erased_ ? EraseStep::Complete : EraseStep::Idle;
        }
        if (state_ == RecorderState::Uploading) {
            return EraseStep::InProgress;
        }
        const uint32_t erase_span = erase_span_end() - base_address_;
        for (uint8_t n = 0U; n < max_sectors && (erase_cursor_ + kSectorSize) <= erase_span; ++n) {
            if (!flash_.erase_4k(base_address_ + erase_cursor_)) {
                if (++erase_fail_streak_ < kEraseSectorRetries) {
                    return EraseStep::InProgress;
                }
                erase_active_ = false;
                region_erased_ = false;
                return EraseStep::Failed;
            }
            erase_fail_streak_ = 0U;
            erase_cursor_ += kSectorSize;
        }
        if (erase_cursor_ >= erase_span) {
            erase_active_ = false;
            region_erased_ = true;
            return EraseStep::Complete;
        }
        return EraseStep::InProgress;
    }

    /* The eraser covers the session region plus the recovery sector when the
     * header lives there, so both header slots are 0xFF for the next session. */
    uint32_t erase_span_end() const {
        if (header_address_ >= base_address_ + capacity_ &&
                header_address_ < base_address_ + capacity_ + kSectorSize) {
            return base_address_ + capacity_ + kSectorSize;
        }
        return base_address_ + capacity_;
    }

    bool erase_in_progress() const { return erase_active_; }
    bool region_pre_erased() const { return region_erased_; }

    /* Delayed-start entry. The caller must have let the background eraser
     * cover the region first: writing a header into unerased flash would trip
     * the driver's read-modify-write path for every batch. */
    bool start(uint16_t node_id, uint32_t session_id, uint64_t start_timestamp_us, uint32_t duration_ms) {
        if (state_ != RecorderState::Idle && state_ != RecorderState::ReadyForUpload) {
            return false;
        }
        if (!region_erased_ || erase_active_) {
            return false;
        }
        reset_runtime();
        configure_header(node_id, session_id, start_timestamp_us, duration_ms);
        state_ = RecorderState::Recording;
        return flash_.write(header_address_, &header_, sizeof(header_));
    }

    /* Prepare entry: RAM-arms the session without touching flash. The header
     * is written by finish_prepare_header() once the eraser reports the region
     * ready, so the multi-second erase never blocks the BLE handler. */
    bool prepare(uint16_t node_id, uint32_t session_id, uint64_t start_timestamp_us, uint32_t duration_ms) {
        if (state_ != RecorderState::Idle && state_ != RecorderState::ReadyForUpload) {
            return false;
        }
        reset_runtime();
        configure_header(node_id, session_id, start_timestamp_us, duration_ms);
        state_ = RecorderState::Armed;
        header_write_pending_ = true;
        return true;
    }

    bool finish_prepare_header() {
        if (state_ != RecorderState::Armed || !header_write_pending_) {
            return false;
        }
        if (erase_active_ || !region_erased_) {
            return false;
        }
        if (!flash_.write(header_address_, &header_, sizeof(header_))) {
            return false;
        }
        header_write_pending_ = false;
        region_erased_ = false;
        return true;
    }

    bool start_prepared(uint16_t node_id, uint32_t session_id, uint64_t start_timestamp_us, uint32_t duration_ms) {
        if (state_ != RecorderState::Armed || header_write_pending_) {
            return false;
        }
        if (header_.node_id != node_id ||
            header_.session_id != session_id ||
            header_.start_timestamp_us != start_timestamp_us ||
            header_.requested_duration_ms != duration_ms) {
            configure_header(node_id, session_id, start_timestamp_us, duration_ms);
        }
        state_ = RecorderState::Recording;
        return flash_.write(header_address_, &header_, sizeof(header_));
    }

    void cancel_prepared() {
        if (state_ == RecorderState::Armed) {
            reset_runtime();
        }
    }

    bool prepared(uint32_t session_id) const {
        return state_ == RecorderState::Armed && header_.session_id == session_id;
    }

    bool append_bno85(const Bno85Sample &sample) {
        return append_sample(sample, bno85_cursor_, bno85_limit(), header_.bno85_sample_count, header_.bno85_payload_size);
    }

    bool append_icm45686(const Icm45686Sample &sample) {
        return append_sample(sample, icm45686_cursor_, base_address_ + capacity_, header_.icm45686_sample_count,
                             header_.icm45686_payload_size);
    }

    bool append_bno85_batch(const Bno85Sample *samples, uint32_t count) {
        return append_samples(samples, count, bno85_cursor_, bno85_limit(), header_.bno85_sample_count,
                              header_.bno85_payload_size);
    }

    bool append_icm45686_batch(const Icm45686Sample *samples, uint32_t count) {
        return append_samples(samples, count, icm45686_cursor_, base_address_ + capacity_, header_.icm45686_sample_count,
                              header_.icm45686_payload_size);
    }

    bool finalize(uint32_t actual_duration_ms) {
        if (state_ != RecorderState::Recording) {
            return false;
        }
        state_ = RecorderState::Finalizing;
        header_.actual_duration_ms = actual_duration_ms;
        header_.completion_flag = kSessionComplete;
        uint32_t ordered_payload_crc = 0U;
        if (!crc_flash_region(bno85_payload_address(), header_.bno85_payload_size, ordered_payload_crc) ||
            !crc_flash_region(icm45686_payload_address(), header_.icm45686_payload_size, ordered_payload_crc)) {
            return false;
        }
        header_.payload_crc32 = ordered_payload_crc;
        header_.header_crc32 = session_header_crc(header_);
        /* Slot B is still 0xFF after the session-start region erase, so this
         * is a single page program: no erase window, and the payload sectors
         * are never touched by finalization. A crash before this write leaves
         * the incomplete slot A header; a crash after it is recoverable at
         * boot via recover_after_boot(). */
        if (!flash_.write(finalized_header_address(), &header_, sizeof(header_))) {
            return false;
        }
        state_ = RecorderState::ReadyForUpload;
        return true;
    }

    bool begin_upload() {
        if (state_ != RecorderState::ReadyForUpload) {
            return false;
        }
        state_ = RecorderState::Uploading;
        return true;
    }

    bool mark_transfer_complete() {
        if (state_ != RecorderState::Uploading) {
            return false;
        }
        state_ = RecorderState::AwaitingAck;
        return true;
    }

    /* Post-upload erase: state advances immediately (the caller ACKs at once)
     * and the erase itself runs chunked from the node superloop, leaving the
     * region pre-erased for the next session. */
    bool acknowledge_and_begin_erase() {
        if (state_ != RecorderState::AwaitingAck) {
            return false;
        }
        state_ = RecorderState::EraseAfterAck;
        return begin_region_erase() != EraseStep::Failed;
    }

    bool erase_after_ack_done() {
        if (state_ != RecorderState::EraseAfterAck) {
            return false;
        }
        reset_runtime();
        region_erased_ = true;
        return true;
    }

    bool erase_after_ack_failed() {
        if (state_ != RecorderState::EraseAfterAck) {
            return false;
        }
        reset_runtime();
        return true;
    }

    bool force_reset_and_background_erase() {
        reset_runtime();
        return begin_region_erase() != EraseStep::Failed;
    }

    uint32_t total_size() const {
        return sizeof(SessionHeader) + header_.bno85_payload_size + header_.icm45686_payload_size;
    }

    uint32_t bno85_payload_address() const { return base_address_ + sizeof(SessionHeader); }
    uint32_t icm45686_payload_address() const { return base_address_ + sizeof(SessionHeader) + bno85_region_size_; }

private:
    void configure_header(uint16_t node_id, uint32_t session_id, uint64_t start_timestamp_us, uint32_t duration_ms) {
        header_.magic = kSessionMagic;
        header_.version = kSessionFormatVersion;
        header_.node_id = node_id;
        header_.session_id = session_id;
        header_.start_timestamp_us = start_timestamp_us;
        header_.requested_duration_ms = duration_ms;
        header_.sensor_mask = kSensorBno85 | kSensorIcm45686;
        bno85_cursor_ = base_address_ + sizeof(SessionHeader);
        icm45686_cursor_ = bno85_cursor_ + bno85_region_size_;
    }

    bool crc_flash_region(uint32_t address, uint32_t size, uint32_t &crc) {
        uint8_t buffer[128] = {0U};
        uint32_t offset = 0U;
        while (offset < size) {
            const uint32_t remaining = size - offset;
            const uint32_t chunk = remaining > sizeof(buffer) ? static_cast<uint32_t>(sizeof(buffer)) : remaining;
            if (!flash_.read(address + offset, buffer, chunk)) {
                return false;
            }
            crc = crc32_update(crc, buffer, chunk);
            offset += chunk;
        }
        return true;
    }

    template <typename Sample>
    bool append_sample(const Sample &sample, uint32_t &cursor, uint32_t limit, uint32_t &count, uint32_t &payload_size) {
        return append_samples(&sample, 1U, cursor, limit, count, payload_size);
    }

    template <typename Sample>
    bool append_samples(const Sample *samples, uint32_t sample_count, uint32_t &cursor, uint32_t limit,
                        uint32_t &count, uint32_t &payload_size) {
        if (state_ != RecorderState::Recording) {
            return false;
        }
        if (samples == nullptr || sample_count == 0U) {
            return false;
        }
        const uint32_t bytes = sample_count * static_cast<uint32_t>(sizeof(Sample));
        if ((cursor + bytes) < cursor || (cursor + bytes) > limit) {
            return false;
        }
        /* Payload regions are pre-erased by the background eraser before the
         * header is written, so use the no-RMW fast path. */
        if (!flash_.write_pre_erased(cursor, samples, bytes)) {
            return false;
        }
        cursor += bytes;
        count += sample_count;
        payload_size += bytes;
        return true;
    }

    void reset_runtime() {
        memset(&header_, 0, sizeof(header_));
        state_ = RecorderState::Idle;
        bno85_cursor_ = base_address_ + sizeof(SessionHeader);
        icm45686_cursor_ = bno85_cursor_ + bno85_region_size_;
        header_write_pending_ = false;
        region_erased_ = false;
    }

    uint32_t bno85_limit() const {
        return base_address_ + sizeof(SessionHeader) + bno85_region_size_;
    }

    SessionFlash &flash_;
    const uint32_t base_address_;
    uint32_t capacity_;
    uint32_t bno85_region_size_;
    uint32_t header_address_;
    RecorderState state_;
    SessionHeader header_;
    uint32_t bno85_cursor_;
    uint32_t icm45686_cursor_;
    static constexpr uint8_t kEraseSectorRetries = 3U;
    bool erase_active_ = false;
    bool region_erased_ = false;
    bool header_write_pending_ = false;
    uint32_t erase_cursor_ = 0U;
    uint8_t erase_fail_streak_ = 0U;
};

} // namespace exo

#endif

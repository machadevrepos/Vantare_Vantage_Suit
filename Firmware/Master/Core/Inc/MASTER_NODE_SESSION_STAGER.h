#ifndef MASTER_NODE_SESSION_STAGER_H_
#define MASTER_NODE_SESSION_STAGER_H_

#include <stddef.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>

#include "ff.h"
#include <BLE_RECORD_PROTOCOL.h>
#include <RECORDING_TYPES.h>

namespace exo {
namespace node_session_staging {

enum class NodeSessionStageOperation : uint8_t {
    None,
    InvalidDone,
    AlreadyActive,
    OpenWrite,
    WrongSource,
    WrongSession,
    InvalidChunk,
    Gap,
    PartialOverlap,
    DuplicateSeek,
    DuplicateRead,
    DuplicateMismatch,
    WriteSeek,
    Write,
    WriteShort,
    Incomplete,
    CloseWrite,
    OpenRead,
    HeaderSeek,
    HeaderRead,
    HeaderInvalid,
    SizeInvalid,
    PayloadSeek,
    PayloadRead,
    PayloadCrcInvalid,
    SampleSeek,
    SampleRead,
    CloseRead,
    ShutdownClose,
    Unlink
};

enum class NodeSessionValidationStatus : uint8_t {
    Idle,
    InProgress,
    ReadyToFinalize,
    Complete,
    Failed
};

struct NodeSessionFatFsOps {
    FRESULT (*open_fn)(FIL *file, const TCHAR *path, BYTE mode);
    FRESULT (*write_fn)(FIL *file, const void *data, UINT bytes, UINT *written);
    FRESULT (*read_fn)(FIL *file, void *data, UINT bytes, UINT *read);
    FRESULT (*lseek_fn)(FIL *file, FSIZE_t offset);
    FRESULT (*close_fn)(FIL *file);
    FRESULT (*unlink_fn)(const TCHAR *path);
};

static const NodeSessionFatFsOps kDefaultNodeSessionFatFsOps = {
    f_open, f_write, f_read, f_lseek, f_close, f_unlink
};

} // namespace node_session_staging

class MasterNodeSessionStager {
public:
    explicit MasterNodeSessionStager(
            const node_session_staging::NodeSessionFatFsOps *ops = nullptr)
        : ops_(ops != nullptr ? ops : &node_session_staging::kDefaultNodeSessionFatFsOps)
    {
    }

    bool begin(const RecordDoneMessage &done, uint16_t file_index = 1U)
    {
        if (file_open_) {
            return set_error(node_session_staging::NodeSessionStageOperation::AlreadyActive,
                    FR_INVALID_OBJECT);
        }
        reset_state();
        if (done.command != RecordCommand::RecordDone || done.node_id < 1U ||
                done.node_id > 4U || done.total_size < sizeof(SessionHeader)) {
            return set_error(node_session_staging::NodeSessionStageOperation::InvalidDone,
                    FR_INVALID_PARAMETER);
        }
        done_ = done;
        make_staging_path(file_index, static_cast<uint8_t>(done.node_id));
        const FRESULT result = ops_->open_fn(&file_, staging_path(),
                static_cast<BYTE>(FA_CREATE_ALWAYS | FA_WRITE | FA_READ));
        if (result != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::OpenWrite, result);
        }
        file_open_ = true;
        writable_ = true;
        active_ = true;
        stage_started_ = true;
        read_cursor_ = kInvalidCursor;
        return true;
    }

    bool accept_chunk(uint8_t source_id, uint32_t session_id, uint32_t byte_offset,
            const uint8_t *data, uint16_t size)
    {
        if (!active_ || !writable_ || !file_open_) {
            return set_error(node_session_staging::NodeSessionStageOperation::InvalidChunk,
                    FR_INVALID_OBJECT);
        }
        if (source_id != done_.node_id) {
            return set_error(node_session_staging::NodeSessionStageOperation::WrongSource,
                    FR_INVALID_PARAMETER);
        }
        if (session_id != done_.session_id) {
            return set_error(node_session_staging::NodeSessionStageOperation::WrongSession,
                    FR_INVALID_PARAMETER);
        }
        if (data == nullptr || size == 0U ||
                static_cast<uint64_t>(byte_offset) + size > done_.total_size) {
            return set_error(node_session_staging::NodeSessionStageOperation::InvalidChunk,
                    FR_INVALID_PARAMETER);
        }
        if (byte_offset > staged_size_) {
            return set_error(node_session_staging::NodeSessionStageOperation::Gap,
                    FR_INVALID_PARAMETER);
        }
        const uint32_t end = byte_offset + size;
        if (byte_offset < staged_size_) {
            if (end > staged_size_) {
                return set_error(node_session_staging::NodeSessionStageOperation::PartialOverlap,
                        FR_INVALID_PARAMETER);
            }
            return duplicate_matches(byte_offset, data, size);
        }

        /* Append path (byte_offset == staged_size_): accumulate in RAM and
         * write larger blocks. accept_chunk runs in BLE event context, so a
         * per-chunk f_lseek+f_write serializes every radio event behind SD
         * latency; buffering amortizes one write across many chunks.
         * staged_size_ advances per copy so a flush triggered mid-chunk sees
         * the invariant "buffer holds [staged_size_ - len, staged_size_)". */
        uint16_t pending = size;
        const uint8_t *cursor = data;
        while (pending > 0U) {
            if (write_buffer_len_ == sizeof(write_buffer_)) {
                if (!flush_write_buffer()) {
                    return false;
                }
            }
            const uint16_t space = static_cast<uint16_t>(sizeof(write_buffer_) -
                    write_buffer_len_);
            const uint16_t take = pending < space ? pending : space;
            memcpy(&write_buffer_[write_buffer_len_], cursor, take);
            write_buffer_len_ = static_cast<uint16_t>(write_buffer_len_ + take);
            staged_size_ += take;
            cursor += take;
            pending = static_cast<uint16_t>(pending - take);
        }
        clear_error();
        return true;
    }

    bool begin_validation()
    {
        validation_status_ = node_session_staging::NodeSessionValidationStatus::Failed;
        if (!active_ || !writable_ || !file_open_ || staged_size_ != done_.total_size) {
            return set_error(node_session_staging::NodeSessionStageOperation::Incomplete,
                    FR_INVALID_OBJECT);
        }
        if (!flush_write_buffer()) {
            return false;
        }
        FRESULT result = ops_->close_fn(&file_);
        read_cursor_ = kInvalidCursor;
        if (result != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::CloseWrite, result);
        }
        file_open_ = false;
        writable_ = false;

        result = ops_->open_fn(&file_, staging_path(), FA_READ);
        if (result != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::OpenRead, result);
        }
        file_open_ = true;
        read_cursor_ = kInvalidCursor;
        if (!read_exact(0U, &header_, sizeof(header_),
                node_session_staging::NodeSessionStageOperation::HeaderSeek,
                node_session_staging::NodeSessionStageOperation::HeaderRead)) {
            return close_invalid_stage(last_operation_, last_result_);
        }
        if (header_.magic != kSessionMagic || header_.version != kSessionFormatVersion ||
                header_.node_id != done_.node_id || header_.session_id != done_.session_id ||
                header_.completion_flag != kSessionComplete ||
                header_.header_crc32 != session_header_crc(header_)) {
            return close_invalid_stage(
                    node_session_staging::NodeSessionStageOperation::HeaderInvalid,
                    FR_INVALID_PARAMETER);
        }

        const uint64_t expected_bno = static_cast<uint64_t>(header_.bno85_sample_count) *
                sizeof(Bno85Sample);
        const uint64_t expected_icm = static_cast<uint64_t>(header_.icm45686_sample_count) *
                sizeof(Icm45686Sample);
        const uint64_t expected_total = sizeof(SessionHeader) + expected_bno + expected_icm;
        if (expected_bno != header_.bno85_payload_size ||
                expected_icm != header_.icm45686_payload_size ||
                expected_total != done_.total_size) {
            return close_invalid_stage(
                    node_session_staging::NodeSessionStageOperation::SizeInvalid,
                    FR_INVALID_PARAMETER);
        }

        result = ops_->lseek_fn(&file_, static_cast<FSIZE_t>(sizeof(SessionHeader)));
        if (result != FR_OK) {
            read_cursor_ = kInvalidCursor;
            return close_invalid_stage(
                    node_session_staging::NodeSessionStageOperation::PayloadSeek, result);
        }
        read_cursor_ = static_cast<uint32_t>(sizeof(SessionHeader));
        validation_crc_ = 0U;
        validation_remaining_ = done_.total_size - static_cast<uint32_t>(sizeof(SessionHeader));
        validation_status_ = validation_remaining_ == 0U ?
                node_session_staging::NodeSessionValidationStatus::ReadyToFinalize :
                node_session_staging::NodeSessionValidationStatus::InProgress;
        clear_error();
        return true;
    }

    bool step_validation(uint32_t byte_budget)
    {
        if (validation_status_ != node_session_staging::NodeSessionValidationStatus::InProgress ||
                byte_budget == 0U || validation_remaining_ == 0U || !file_open_) {
            return set_error(node_session_staging::NodeSessionStageOperation::PayloadRead,
                    FR_INVALID_PARAMETER);
        }
        uint32_t request_bytes = validation_remaining_;
        if (request_bytes > byte_budget) request_bytes = byte_budget;
        if (request_bytes > sizeof(io_buffer_)) request_bytes = sizeof(io_buffer_);
        const UINT request = static_cast<UINT>(request_bytes);
        UINT received = 0U;
        const FRESULT result = ops_->read_fn(&file_, io_buffer_, request, &received);
        if (result != FR_OK || received != request) {
            read_cursor_ = kInvalidCursor;
            return close_invalid_stage(
                    node_session_staging::NodeSessionStageOperation::PayloadRead, result);
        }
        if (read_cursor_ != kInvalidCursor) {
            read_cursor_ += request;
        }
        validation_crc_ = crc32_update(validation_crc_, io_buffer_, request);
        validation_remaining_ -= request;
        if (validation_remaining_ == 0U) {
            validation_status_ =
                    node_session_staging::NodeSessionValidationStatus::ReadyToFinalize;
        }
        clear_error();
        return true;
    }

    bool finalize_validation()
    {
        if (validation_status_ !=
                node_session_staging::NodeSessionValidationStatus::ReadyToFinalize) {
            return set_error(node_session_staging::NodeSessionStageOperation::Incomplete,
                    FR_INVALID_OBJECT);
        }
        if (validation_crc_ != header_.payload_crc32 ||
                validation_crc_ != done_.payload_crc32) {
            return close_invalid_stage(
                    node_session_staging::NodeSessionStageOperation::PayloadCrcInvalid,
                    FR_INVALID_PARAMETER);
        }
        validated_ = true;
        validation_status_ = node_session_staging::NodeSessionValidationStatus::Complete;
        clear_error();
        return true;
    }

    bool read_bno(uint32_t index, Bno85Sample &out)
    {
        if (!validated_ || index >= header_.bno85_sample_count) return false;
        const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
                index * static_cast<uint32_t>(sizeof(Bno85Sample));
        return read_exact(offset, &out, sizeof(out),
                node_session_staging::NodeSessionStageOperation::SampleSeek,
                node_session_staging::NodeSessionStageOperation::SampleRead);
    }

    bool read_icm(uint32_t index, Icm45686Sample &out)
    {
        if (!validated_ || index >= header_.icm45686_sample_count) return false;
        const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
                header_.bno85_payload_size +
                index * static_cast<uint32_t>(sizeof(Icm45686Sample));
        return read_exact(offset, &out, sizeof(out),
                node_session_staging::NodeSessionStageOperation::SampleSeek,
                node_session_staging::NodeSessionStageOperation::SampleRead);
    }

    bool discard_after_success()
    {
        if (!validated_ || discarded_) return false;
        if (file_open_) {
            const FRESULT close_result = ops_->close_fn(&file_);
            read_cursor_ = kInvalidCursor;
            if (close_result != FR_OK) {
                return set_error(node_session_staging::NodeSessionStageOperation::CloseRead,
                        close_result);
            }
            file_open_ = false;
        }
        discarded_ = true;
        stage_started_ = false;
        active_ = false;
        clear_error();
        return true;
    }

    bool shutdown()
    {
        if (file_open_) {
            /* Best effort: shutdown is also the abandon path, so the staged
             * file must be closed even if the tail cannot be written. */
            (void)flush_write_buffer();
            const FRESULT close_result = ops_->close_fn(&file_);
            read_cursor_ = kInvalidCursor;
            if (close_result != FR_OK) {
                return set_error(node_session_staging::NodeSessionStageOperation::ShutdownClose,
                        close_result);
            }
            file_open_ = false;
        }
        active_ = false;
        writable_ = false;
        validated_ = false;
        validation_status_ = node_session_staging::NodeSessionValidationStatus::Idle;
        validation_remaining_ = 0U;
        validation_crc_ = 0U;
        read_cursor_ = kInvalidCursor;
        clear_error();
        return true;
    }

    /* Abandon the stage and remove the truncated file. An abandoned or failed
     * upload must leave no R####N#.BIN artifact: the run index stays reusable
     * and no partial file can be mistaken for a completed archive. A validated
     * stage is never unlinked (it is the durable archive), and the node's own
     * flash copy is unaffected because discarded_ stays clear — the session can
     * still be re-pulled later. */
    bool abandon_and_unlink()
    {
        /* shutdown() clears validated_, so capture it first: a validated stage
         * is the durable archive and must never be unlinked. */
        const bool was_validated = validated_;
        const bool close_ok = shutdown();
        if (!stage_started_ || was_validated) {
            return close_ok;
        }
        stage_started_ = false;
        if (!close_ok && file_open_) {
            (void)ops_->close_fn(&file_);
            file_open_ = false;
        }
        const FRESULT result = ops_->unlink_fn(staging_path_);
        if (result != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::Unlink, result);
        }
        return close_ok;
    }

    bool active() const { return active_; }
    bool validated() const { return validated_; }
    bool node_flash_may_be_erased() const { return discarded_; }
    uint32_t staged_size() const { return staged_size_; }
    uint32_t validation_remaining() const { return validation_remaining_; }
    node_session_staging::NodeSessionValidationStatus validation_status() const
    {
        return validation_status_;
    }
    const SessionHeader &header() const { return header_; }
    node_session_staging::NodeSessionStageOperation last_operation() const
    {
        return last_operation_;
    }
    FRESULT last_result() const { return last_result_; }

private:
    static constexpr uint32_t kInvalidCursor = UINT32_MAX;

    const char *staging_path() const { return staging_path_; }

    void make_staging_path(uint16_t index, uint8_t node_id)
    {
        static const char prefix[] = "/SESSIONS/R";
        static const char suffix[] = ".BIN";
        size_t cursor = 0U;
        for (size_t i = 0U; i < sizeof(prefix) - 1U; ++i) staging_path_[cursor++] = prefix[i];
        staging_path_[cursor++] = static_cast<char>('0' + ((index / 1000U) % 10U));
        staging_path_[cursor++] = static_cast<char>('0' + ((index / 100U) % 10U));
        staging_path_[cursor++] = static_cast<char>('0' + ((index / 10U) % 10U));
        staging_path_[cursor++] = static_cast<char>('0' + (index % 10U));
        staging_path_[cursor++] = 'N';
        staging_path_[cursor++] = static_cast<char>('0' + node_id);
        for (size_t i = 0U; i < sizeof(suffix) - 1U; ++i) staging_path_[cursor++] = suffix[i];
        staging_path_[cursor] = '\0';
    }

    void reset_state()
    {
        memset(&file_, 0, sizeof(file_));
        memset(&done_, 0, sizeof(done_));
        memset(&header_, 0, sizeof(header_));
        staged_size_ = 0U;
        write_buffer_len_ = 0U;
        active_ = false;
        writable_ = false;
        file_open_ = false;
        validated_ = false;
        discarded_ = false;
        stage_started_ = false;
        validation_remaining_ = 0U;
        validation_crc_ = 0U;
        read_cursor_ = kInvalidCursor;
        validation_status_ = node_session_staging::NodeSessionValidationStatus::Idle;
        clear_error();
    }

    /* Write the buffered tail at its absolute offset. Every staged write goes
     * through here, so the file cursor never carries state between calls. */
    bool flush_write_buffer()
    {
        if (write_buffer_len_ == 0U) {
            return true;
        }
        const uint32_t buffer_start = staged_size_ - write_buffer_len_;
        const FRESULT seek = ops_->lseek_fn(&file_, static_cast<FSIZE_t>(buffer_start));
        read_cursor_ = kInvalidCursor;
        if (seek != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::WriteSeek, seek);
        }
        UINT written = 0U;
        const FRESULT result = ops_->write_fn(&file_, write_buffer_, write_buffer_len_, &written);
        if (result != FR_OK) {
            return set_error(node_session_staging::NodeSessionStageOperation::Write, result);
        }
        if (written != write_buffer_len_) {
            return set_error(node_session_staging::NodeSessionStageOperation::WriteShort, FR_OK);
        }
        write_buffer_len_ = 0U;
        return true;
    }

    bool duplicate_matches(uint32_t offset, const uint8_t *data, uint16_t size)
    {
        /* The tail held in the RAM write buffer is compared without I/O; only
         * the already-flushed prefix is read back from the file. */
        const uint32_t buffer_start = staged_size_ - write_buffer_len_;
        uint32_t cursor = offset;
        const uint8_t *expected = data;
        uint32_t remaining = size;
        if (cursor < buffer_start) {
            const FRESULT seek = ops_->lseek_fn(&file_, static_cast<FSIZE_t>(cursor));
            read_cursor_ = kInvalidCursor;
            if (seek != FR_OK) {
                return set_error(node_session_staging::NodeSessionStageOperation::DuplicateSeek,
                        seek);
            }
            uint32_t flushed_bytes = buffer_start - cursor;
            /* Never read past the caller's buffer: a duplicate that ends
             * before the flush point only compares its own length. */
            if (flushed_bytes > remaining) {
                flushed_bytes = remaining;
            }
            while (flushed_bytes > 0U) {
                const UINT request = flushed_bytes < sizeof(io_buffer_) ?
                        static_cast<UINT>(flushed_bytes) : static_cast<UINT>(sizeof(io_buffer_));
                UINT received = 0U;
                const FRESULT result = ops_->read_fn(&file_, io_buffer_, request, &received);
                if (result != FR_OK || received != request) {
                    return set_error(node_session_staging::NodeSessionStageOperation::DuplicateRead,
                            result);
                }
                if (memcmp(io_buffer_, expected, request) != 0) {
                    return set_error(
                            node_session_staging::NodeSessionStageOperation::DuplicateMismatch,
                            FR_INVALID_PARAMETER);
                }
                flushed_bytes -= request;
                cursor += request;
                expected += request;
                remaining -= static_cast<uint16_t>(request);
            }
        }
        if (remaining > 0U) {
            if (memcmp(&write_buffer_[cursor - buffer_start], expected, remaining) != 0) {
                return set_error(
                        node_session_staging::NodeSessionStageOperation::DuplicateMismatch,
                        FR_INVALID_PARAMETER);
            }
        }
        clear_error();
        return true;
    }

    bool read_exact(uint32_t offset, void *output, size_t size,
            node_session_staging::NodeSessionStageOperation seek_operation,
            node_session_staging::NodeSessionStageOperation read_operation)
    {
        if (!file_open_ || output == nullptr || size > static_cast<size_t>(UINT_MAX)) {
            return set_error(read_operation, FR_INVALID_OBJECT);
        }
        if (read_cursor_ != offset) {
            const FRESULT seek_result = ops_->lseek_fn(&file_, static_cast<FSIZE_t>(offset));
            if (seek_result != FR_OK) {
                read_cursor_ = kInvalidCursor;
                return set_error(seek_operation, seek_result);
            }
            read_cursor_ = offset;
        }
        UINT received = 0U;
        const UINT request = static_cast<UINT>(size);
        const FRESULT read_result = ops_->read_fn(&file_, output, request, &received);
        if (read_result != FR_OK || received != request) {
            read_cursor_ = kInvalidCursor;
            return set_error(read_operation, read_result);
        }
        read_cursor_ += request;
        clear_error();
        return true;
    }

    bool close_invalid_stage(node_session_staging::NodeSessionStageOperation operation,
            FRESULT result)
    {
        last_operation_ = operation;
        last_result_ = result;
        if (file_open_) {
            const FRESULT close_result = ops_->close_fn(&file_);
            read_cursor_ = kInvalidCursor;
            if (close_result != FR_OK) {
                return set_error(node_session_staging::NodeSessionStageOperation::CloseRead,
                        close_result);
            }
            file_open_ = false;
        }
        active_ = false;
        validation_status_ = node_session_staging::NodeSessionValidationStatus::Failed;
        return false;
    }

    bool set_error(node_session_staging::NodeSessionStageOperation operation, FRESULT result)
    {
        last_operation_ = operation;
        last_result_ = result;
        return false;
    }

    void clear_error()
    {
        last_operation_ = node_session_staging::NodeSessionStageOperation::None;
        last_result_ = FR_OK;
    }

    const node_session_staging::NodeSessionFatFsOps *ops_;
    FIL file_{};
    RecordDoneMessage done_{};
    SessionHeader header_{};
    char staging_path_[32]{};
    uint8_t io_buffer_[256]{};
    /* Staged-write accumulator: the tail [staged_size_ - write_buffer_len_,
     * staged_size_) lives here until a full block, a read-back, validation or
     * shutdown forces it to the card. */
    static constexpr uint16_t kWriteBufferBytes = 4096U;
    uint8_t write_buffer_[kWriteBufferBytes]{};
    uint16_t write_buffer_len_ = 0U;
    uint32_t staged_size_ = 0U;
    uint32_t validation_remaining_ = 0U;
    uint32_t validation_crc_ = 0U;
    uint32_t read_cursor_ = kInvalidCursor;
    bool active_ = false;
    bool writable_ = false;
    bool file_open_ = false;
    bool validated_ = false;
    bool discarded_ = false;
    /* True from a successful begin() until the stage is either validated (the
     * file becomes the durable archive) or unlinked. Guards abandon_and_unlink
     * against touching files it does not own. */
    bool stage_started_ = false;
    node_session_staging::NodeSessionValidationStatus validation_status_ =
            node_session_staging::NodeSessionValidationStatus::Idle;
    node_session_staging::NodeSessionStageOperation last_operation_ =
            node_session_staging::NodeSessionStageOperation::None;
    FRESULT last_result_ = FR_OK;
};

} // namespace exo

#endif /* MASTER_NODE_SESSION_STAGER_H_ */

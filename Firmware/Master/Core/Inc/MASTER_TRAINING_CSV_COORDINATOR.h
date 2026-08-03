#ifndef MASTER_TRAINING_CSV_COORDINATOR_H_
#define MASTER_TRAINING_CSV_COORDINATOR_H_

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "app_fatfs.h"
#include <BLE_RECORD_PROTOCOL.h>
#include <MASTER_NODE_SESSION_STAGER.h>
#include <MASTER_SD_SESSION_RECORDER.h>
#include <MASTER_SESSION_TIMESTAMP_LEDGER.h>
#include <MASTER_TRAINING_CSV_LOGGER.h>
#include "blepipe_proto.h"

namespace exo {

enum class TrainingCsvState : uint8_t {
    Idle,
    WaitingForMaster,
    ConvertMasterBno,
    ConvertMasterIcm,
    WaitingForNode,
    ReceiveNode,
    ValidateNode,
    ConvertNodeBno,
    ConvertNodeIcm,
    Complete,
    CsvError,
    StageError
};

/* Exact call site that first drove the session into CsvError/StageError. Latched together with
 * the stager/logger error codes at the moment of failure, because the cleanup that follows
 * (logger shutdown, stager shutdown) resets those codes before anything can report them. */
enum class TrainingFailSite : uint8_t {
    None = 0xFFU,
    Site0 = 0,  /* master header self-consistency        */
    Site1,      /* logger.begin (CSV create)             */
    Site2,      /* stager.begin (staging file create)    */
    Site3,      /* stager.accept_chunk                   */
    Site4,      /* stager.begin_validation               */
    Site5,      /* stager.step_validation                */
    Site6,      /* stager.finalize_validation (CRC)      */
    Site7,      /* validation_status == Failed           */
    Site8,      /* logger.service (periodic sync)        */
    Site9,      /* shutdown: logger file still open      */
    Site10,     /* shutdown: stager still active         */
    Site11,     /* master BNO read/append                */
    Site12,     /* master ICM ledger/read/append         */
    Site13,     /* stager.read_bno                       */
    Site14,     /* logger.append_bno (node)              */
    Site15,     /* stager.read_icm                       */
    Site16,     /* logger.append_icm (node)              */
    Site17,     /* logger.mark_source_complete           */
    Site18,     /* stager.discard_after_success          */
    Site19,     /* logger.shutdown after full conversion */
    Site20,     /* master archive durability              */
    SessionStall
};

namespace training_csv_coordinator {

struct MasterRecordingOps {
    bool (*ready_fn)(const MasterSdSessionRecorder &recorder);
    const SessionHeader &(*header_fn)(const MasterSdSessionRecorder &recorder);
    bool (*read_fn)(MasterSdSessionRecorder &recorder, uint32_t offset,
            void *output, uint32_t size);
};

inline bool default_master_ready(const MasterSdSessionRecorder &recorder)
{
    return recorder.ready();
}

inline const SessionHeader &default_master_header(const MasterSdSessionRecorder &recorder)
{
    return recorder.header();
}

inline bool default_master_read(MasterSdSessionRecorder &recorder, uint32_t offset,
        void *output, uint32_t size)
{
    return recorder.read(offset, output, size);
}

static const MasterRecordingOps kDefaultMasterRecordingOps = {
    default_master_ready, default_master_header, default_master_read
};

} // namespace training_csv_coordinator

class MasterTrainingCsvCoordinator {
public:
    static constexpr uint32_t kRowsPerService = 8U;
    static constexpr uint32_t kValidationBytesPerService = 256U;
    /* Matches the desktop tool's session watchdog so both give up on a stalled Node together. */
    static constexpr uint32_t kSessionStallMs = 30000U;

    explicit MasterTrainingCsvCoordinator(
            const training_csv::TrainingCsvFatFsOps *logger_ops = nullptr,
            const node_session_staging::NodeSessionFatFsOps *stager_ops = nullptr,
            const training_csv_coordinator::MasterRecordingOps *master_ops = nullptr)
        : logger_(logger_ops), stager_(stager_ops),
          master_ops_(master_ops != nullptr ? master_ops :
                  &training_csv_coordinator::kDefaultMasterRecordingOps)
    {
    }

    bool begin_session(uint32_t session_id, uint8_t expected_source_mask)
    {
        /* A finalized failure has retained its incomplete TMP and released its handles, so it
         * must not block the next recording the way an in-flight session does. */
        const bool recoverable_error = partial_finalized_ &&
                (state_ == TrainingCsvState::CsvError || state_ == TrainingCsvState::StageError);
        if (state_ != TrainingCsvState::Idle && state_ != TrainingCsvState::Complete &&
                !recoverable_error) {
            return false;
        }
        if (logger_.has_open_file() || stager_.active()) {
            return false;
        }
        if ((expected_source_mask & 0x01U) == 0U ||
                (expected_source_mask & static_cast<uint8_t>(~0x1FU)) != 0U) {
            return false;
        }
        clear_failure();
        last_progress_ms_ = 0U;
        progress_seq_ = 0U;
        last_progress_seq_ = 0U;
        partial_finalized_ = false;
        active_session_id_ = session_id;
        expected_source_mask_ = expected_source_mask;
        completed_source_mask_ = 0U;
        active_node_id_ = 0U;
        master_bno_index_ = 0U;
        master_icm_index_ = 0U;
        node_bno_index_ = 0U;
        node_icm_index_ = 0U;
        master_ledger_matches_ = false;
        memset(&master_header_, 0, sizeof(master_header_));
        ledger_.reset(session_id);
        state_ = TrainingCsvState::WaitingForMaster;
        return true;
    }

    bool note_master_icm_time(uint32_t session_id, uint32_t session_time_us)
    {
        if (session_id != active_session_id_ ||
                (state_ != TrainingCsvState::WaitingForMaster &&
                 state_ != TrainingCsvState::ConvertMasterBno &&
                 state_ != TrainingCsvState::ConvertMasterIcm)) {
            return false;
        }
        return ledger_.append_icm(session_time_us);
    }

    bool cancel_session()
    {
        if (state_ != TrainingCsvState::WaitingForMaster || logger_.has_open_file() ||
                stager_.active()) {
            return false;
        }
        active_session_id_ = 0U;
        expected_source_mask_ = 0U;
        completed_source_mask_ = 0U;
        active_node_id_ = 0U;
        master_bno_index_ = 0U;
        master_icm_index_ = 0U;
        node_bno_index_ = 0U;
        node_icm_index_ = 0U;
        master_ledger_matches_ = false;
        memset(&master_header_, 0, sizeof(master_header_));
        ledger_.reset(0U);
        last_progress_ms_ = 0U;
        progress_seq_ = 0U;
        last_progress_seq_ = 0U;
        partial_finalized_ = false;
        state_ = TrainingCsvState::Idle;
        return true;
    }

    void on_master_finalized(MasterSdSessionRecorder &recorder)
    {
        if (state_ != TrainingCsvState::WaitingForMaster || !master_ops_->ready_fn(recorder) ||
                master_ops_->header_fn(recorder).session_id != active_session_id_) {
            return;
        }
        master_header_ = master_ops_->header_fn(recorder);
        /* Same reasoning as the Node stager header gate: a missed sensor tick is a capture
         * quality issue, not a corrupt session. Only self-consistency is required here, so a
         * degraded run still converts instead of yielding no CSV at all. */
        if (master_header_.bno85_sample_count != master_header_.bno85_captured_count ||
                master_header_.icm45686_sample_count != master_header_.icm45686_captured_count) {
            fail(TrainingCsvState::CsvError, TrainingFailSite::Site0);
            return;
        }
        master_ledger_matches_ = !ledger_.overflowed() &&
                ledger_.session_id() == active_session_id_ &&
                ledger_.icm_count() == master_header_.icm45686_sample_count;
        if (!logger_.begin(active_session_id_, expected_source_mask_, 0U)) {
            fail(TrainingCsvState::CsvError, TrainingFailSite::Site1);
            return;
        }
        state_ = TrainingCsvState::ConvertMasterBno;
    }

    void on_node_record_done(const RecordDoneMessage &done)
    {
        if (partial_finalized_) {
            /* The incomplete CSV for this session is already closed and retained as TMP. */
            return;
        }
        if (state_ != TrainingCsvState::WaitingForNode ||
                done.command != RecordCommand::RecordDone ||
                done.session_id != active_session_id_ || done.node_id < 1U || done.node_id > 4U ||
                (expected_source_mask_ & static_cast<uint8_t>(1U << done.node_id)) == 0U ||
                (completed_source_mask_ & static_cast<uint8_t>(1U << done.node_id)) != 0U) {
            return;
        }
        if (!stager_.begin(done, logger_.file_index())) {
            fail(TrainingCsvState::StageError, TrainingFailSite::Site2);
            return;
        }
        active_node_id_ = static_cast<uint8_t>(done.node_id);
        node_bno_index_ = 0U;
        node_icm_index_ = 0U;
        state_ = TrainingCsvState::ReceiveNode;
    }

    void on_node_reliable_frame(uint8_t node_id, const uint8_t *frame, uint16_t length)
    {
        // Validate frame: must be a RecordReliableFrame for our session
        if (partial_finalized_ || frame == nullptr ||
                length < sizeof(RecordReliableFrameHeader) ||
                node_id < 1U || node_id > 4U) {
            return;
        }
        RecordReliableFrameHeader header{};
        memcpy(&header, frame, sizeof(header));
        if (header.command != RecordCommand::ReliableFrame ||
                header.proto_version != kRecordReliableProtoVersion ||
                header.magic != kRecordReliableMagic ||
                header.frame_type != static_cast<uint8_t>(RecordReliableType::Chunk) ||
                header.source_id != node_id || header.session_id != active_session_id_ ||
                static_cast<size_t>(sizeof(header)) + header.payload_len != length) {
            return;
        }
        const uint8_t *payload = frame + sizeof(header);
        if (blepipe_crc16_ccitt(payload, header.payload_len) != header.payload_crc16) {
            return;
        }

        if (state_ == TrainingCsvState::ReceiveNode) {
            // Normal path: coordinator is ready — write to stager directly
            if (node_id != active_node_id_) {
                return;
            }
            if (!stager_.accept_chunk(node_id, header.session_id, header.byte_offset,
                    payload, header.payload_len)) {
                fail(TrainingCsvState::StageError, TrainingFailSite::Site3);
                return;
            }
            ++progress_seq_;
            if ((header.flags & kRecordFlagFinalChunk) != 0U) {
                state_ = TrainingCsvState::ValidateNode;
            }
        }
        // Frames before ReceiveNode remain queued by the reliable-transfer manifest path.
    }

    void service(MasterSdSessionRecorder &recorder, uint32_t now_ms)
    {
        const TrainingCsvState entry_state = state_;
        service_state(recorder, now_ms);
        if (state_ != entry_state) {
            ++progress_seq_;
        }
        if (progress_seq_ != last_progress_seq_ || last_progress_ms_ == 0U) {
            last_progress_seq_ = progress_seq_;
            last_progress_ms_ = now_ms;
        }
        service_finalize(now_ms);
    }

    /* Flushes and closes a session that cannot finish while retaining TRNxxxx.TMP for
     * later recovery. Incomplete sessions are never renamed and never receive .OK. */
    void finalize_partial(uint32_t now_ms)
    {
        if (partial_finalized_) {
            return;
        }
        partial_finalized_ = true;
        (void)logger_.shutdown(now_ms);
        (void)stager_.shutdown();
        active_node_id_ = 0U;
    }

    bool partial_finalized() const { return partial_finalized_; }

    void service_state(MasterSdSessionRecorder &recorder, uint32_t now_ms)
    {
        switch (state_) {
            case TrainingCsvState::ConvertMasterBno:
                service_master_bno(recorder, now_ms);
                break;
            case TrainingCsvState::ConvertMasterIcm:
                service_master_icm(recorder, now_ms);
                break;
            case TrainingCsvState::ValidateNode:
                switch (stager_.validation_status()) {
                    case node_session_staging::NodeSessionValidationStatus::Idle:
                        if (!stager_.begin_validation()) {
                            fail(TrainingCsvState::StageError, TrainingFailSite::Site4);
                        }
                        break;
                    case node_session_staging::NodeSessionValidationStatus::InProgress:
                        if (!stager_.step_validation(kValidationBytesPerService)) {
                            fail(TrainingCsvState::StageError, TrainingFailSite::Site5);
                        }
                        break;
                    case node_session_staging::NodeSessionValidationStatus::ReadyToFinalize:
                        if (stager_.finalize_validation()) {
                            state_ = TrainingCsvState::ConvertNodeBno;
                        } else {
                            fail(TrainingCsvState::StageError, TrainingFailSite::Site6);
                        }
                        break;
                    case node_session_staging::NodeSessionValidationStatus::Complete:
                        state_ = TrainingCsvState::ConvertNodeBno;
                        break;
                    case node_session_staging::NodeSessionValidationStatus::Failed:
                        fail(TrainingCsvState::StageError, TrainingFailSite::Site7);
                        break;
                }
                break;
            case TrainingCsvState::ConvertNodeBno:
                service_node_bno(now_ms);
                break;
            case TrainingCsvState::ConvertNodeIcm:
                service_node_icm(now_ms);
                break;
            case TrainingCsvState::WaitingForMaster:
            case TrainingCsvState::WaitingForNode:
            case TrainingCsvState::ReceiveNode:
                if (logger_.ready() && !logger_.service(now_ms)) {
                    fail(TrainingCsvState::CsvError, TrainingFailSite::Site8);
                }
                break;
            case TrainingCsvState::Idle:
            case TrainingCsvState::Complete:
            case TrainingCsvState::CsvError:
            case TrainingCsvState::StageError:
                break;
        }
    }

    /* A session that reaches an error state has no live recovery path, and one that stops
     * making progress will not resume on its own. Flush and close it while retaining the TMP
     * artifact so a later controlled session can recover it. */
    void service_finalize(uint32_t now_ms)
    {
        switch (state_) {
            case TrainingCsvState::CsvError:
            case TrainingCsvState::StageError:
                finalize_partial(now_ms);
                break;
            /* Only states that wait on the Node link are watched. The Convert/Validate states
             * are bounded work that ends in a completion or an error state on their own. */
            case TrainingCsvState::WaitingForNode:
            case TrainingCsvState::ReceiveNode:
                if ((now_ms - last_progress_ms_) >= kSessionStallMs) {
                    fail(TrainingCsvState::StageError, TrainingFailSite::SessionStall);
                    finalize_partial(now_ms);
                }
                break;
            default:
                break;
        }
    }

    void shutdown(uint32_t now_ms)
    {
        (void)logger_.shutdown(now_ms);
        (void)stager_.shutdown();
        if (logger_.has_open_file()) {
            fail(TrainingCsvState::CsvError, TrainingFailSite::Site9);
            return;
        }
        if (stager_.active()) {
            fail(TrainingCsvState::StageError, TrainingFailSite::Site10);
            return;
        }
        active_node_id_ = 0U;
        state_ = TrainingCsvState::Idle;
    }

    TrainingCsvState state() const { return state_; }
    uint32_t active_session_id() const { return active_session_id_; }
    uint8_t active_node_id() const { return active_node_id_; }
    uint8_t expected_source_mask() const { return expected_source_mask_; }
    uint8_t completed_source_mask() const { return completed_source_mask_; }
    uint32_t ledger_count() const { return ledger_.icm_count(); }
    uint32_t master_bno_index() const { return master_bno_index_; }
    uint32_t master_icm_index() const { return master_icm_index_; }
    uint32_t node_bno_index() const { return node_bno_index_; }
    uint32_t node_icm_index() const { return node_icm_index_; }
    bool master_ledger_matches() const { return master_ledger_matches_; }
    const MasterTrainingCsvLogger &logger() const { return logger_; }
    const MasterNodeSessionStager &stager() const { return stager_; }
    TrainingFailSite failure_site() const { return failure_site_; }
    uint8_t failure_node_id() const { return failure_node_id_; }
    node_session_staging::NodeSessionStageOperation failure_stager_operation() const
    {
        return failure_stager_operation_;
    }
    FRESULT failure_stager_result() const { return failure_stager_result_; }
    training_csv::TrainingCsvLogOperation failure_csv_operation() const
    {
        return failure_csv_operation_;
    }
    FRESULT failure_csv_result() const { return failure_csv_result_; }
    void fail_durability() { fail(TrainingCsvState::CsvError, TrainingFailSite::Site20); }

private:
    void clear_failure()
    {
        failure_site_ = TrainingFailSite::None;
        failure_node_id_ = 0U;
        failure_stager_operation_ = node_session_staging::NodeSessionStageOperation::None;
        failure_stager_result_ = FR_OK;
        failure_csv_operation_ = training_csv::TrainingCsvLogOperation::None;
        failure_csv_result_ = FR_OK;
    }

    void fail(TrainingCsvState error_state, TrainingFailSite site)
    {
        if (failure_site_ == TrainingFailSite::None) {
            failure_site_ = site;
            failure_node_id_ = active_node_id_;
            failure_stager_operation_ = stager_.last_operation();
            failure_stager_result_ = stager_.last_result();
            failure_csv_operation_ = logger_.last_operation();
            failure_csv_result_ = logger_.last_result();
        }
        state_ = error_state;
    }

    void service_master_bno(MasterSdSessionRecorder &recorder, uint32_t now_ms)
    {
        uint32_t rows = 0U;
        while (rows < kRowsPerService && master_bno_index_ <
                master_header_.bno85_sample_count) {
            Bno85Sample sample{};
            const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
                    master_bno_index_ * static_cast<uint32_t>(sizeof(Bno85Sample));
            if (!master_ops_->read_fn(recorder, offset, &sample, sizeof(sample)) ||
                    !logger_.append_bno(0U, sample.offset_us, sample, false, 0U, now_ms)) {
                fail(TrainingCsvState::CsvError, TrainingFailSite::Site11);
                return;
            }
            ++master_bno_index_;
            ++rows;
        }
        if (master_bno_index_ == master_header_.bno85_sample_count) {
            if (master_ledger_matches_) {
                state_ = TrainingCsvState::ConvertMasterIcm;
            } else {
                // Ledger didn't match: skip ICM conversion (no timestamp data),
                // but mark Master source complete so the source mask progresses.
                complete_source(0U, now_ms, false);
            }
        }
    }

    void service_master_icm(MasterSdSessionRecorder &recorder, uint32_t now_ms)
    {
        uint32_t rows = 0U;
        while (rows < kRowsPerService && master_icm_index_ <
                master_header_.icm45686_sample_count) {
            Icm45686Sample sample{};
            uint32_t session_time_us = 0U;
            const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
                    master_header_.bno85_payload_size +
                    master_icm_index_ * static_cast<uint32_t>(sizeof(Icm45686Sample));
            if (!ledger_.icm_time(master_icm_index_, session_time_us) ||
                    !master_ops_->read_fn(recorder, offset, &sample, sizeof(sample)) ||
                    !logger_.append_icm(0U, session_time_us, sample, now_ms)) {
                fail(TrainingCsvState::CsvError, TrainingFailSite::Site12);
                return;
            }
            ++master_icm_index_;
            ++rows;
        }
        if (master_icm_index_ == master_header_.icm45686_sample_count) {
            complete_source(0U, now_ms, false);
        }
    }

    void service_node_bno(uint32_t now_ms)
    {
        uint32_t rows = 0U;
        while (rows < kRowsPerService && node_bno_index_ < stager_.header().bno85_sample_count) {
            Bno85Sample sample{};
            if (!stager_.read_bno(node_bno_index_, sample)) {
                fail(TrainingCsvState::StageError, TrainingFailSite::Site13);
                return;
            }
            if (!logger_.append_bno(active_node_id_, sample.offset_us, sample, false, 0U,
                    now_ms)) {
                fail(TrainingCsvState::CsvError, TrainingFailSite::Site14);
                return;
            }
            ++node_bno_index_;
            ++rows;
        }
        if (node_bno_index_ == stager_.header().bno85_sample_count) {
            state_ = TrainingCsvState::ConvertNodeIcm;
        }
    }

    void service_node_icm(uint32_t now_ms)
    {
        uint32_t rows = 0U;
        while (rows < kRowsPerService && node_icm_index_ < stager_.header().icm45686_sample_count) {
            Icm45686Sample sample{};
            if (!stager_.read_icm(node_icm_index_, sample)) {
                fail(TrainingCsvState::StageError, TrainingFailSite::Site15);
                return;
            }
            const uint64_t session_time_us = sample.offset_us;
            if (!logger_.append_icm(active_node_id_, session_time_us, sample, now_ms)) {
                fail(TrainingCsvState::CsvError, TrainingFailSite::Site16);
                return;
            }
            ++node_icm_index_;
            ++rows;
        }
        if (node_icm_index_ == stager_.header().icm45686_sample_count) {
            complete_source(active_node_id_, now_ms, true);
        }
    }

    void complete_source(uint8_t source_id, uint32_t now_ms, bool discard_stage)
    {
        if (!logger_.mark_source_complete(source_id, now_ms)) {
            fail(TrainingCsvState::CsvError, TrainingFailSite::Site17);
            return;
        }
        completed_source_mask_ = logger_.completed_source_mask();
        if (discard_stage && !stager_.discard_after_success()) {
            fail(TrainingCsvState::StageError, TrainingFailSite::Site18);
            return;
        }
        active_node_id_ = 0U;
        if ((completed_source_mask_ & expected_source_mask_) == expected_source_mask_) {
            if (logger_.shutdown(now_ms)) {
                state_ = TrainingCsvState::Complete;
            } else {
                fail(TrainingCsvState::CsvError, TrainingFailSite::Site19);
            }
        } else {
            state_ = TrainingCsvState::WaitingForNode;
        }
    }

    MasterTrainingCsvLogger logger_;
    MasterNodeSessionStager stager_;
    MasterSessionTimestampLedger ledger_;
    const training_csv_coordinator::MasterRecordingOps *master_ops_;
    SessionHeader master_header_{};
    uint32_t active_session_id_ = 0U;
    uint32_t master_bno_index_ = 0U;
    uint32_t master_icm_index_ = 0U;
    uint32_t node_bno_index_ = 0U;
    uint32_t node_icm_index_ = 0U;
    uint8_t expected_source_mask_ = 0U;
    uint8_t completed_source_mask_ = 0U;
    uint8_t active_node_id_ = 0U;
    bool master_ledger_matches_ = false;
    uint32_t last_progress_ms_ = 0U;
    uint32_t progress_seq_ = 0U;
    uint32_t last_progress_seq_ = 0U;
    bool partial_finalized_ = false;
    TrainingFailSite failure_site_ = TrainingFailSite::None;
    uint8_t failure_node_id_ = 0U;
    node_session_staging::NodeSessionStageOperation failure_stager_operation_ =
            node_session_staging::NodeSessionStageOperation::None;
    FRESULT failure_stager_result_ = FR_OK;
    training_csv::TrainingCsvLogOperation failure_csv_operation_ =
            training_csv::TrainingCsvLogOperation::None;
    FRESULT failure_csv_result_ = FR_OK;
    TrainingCsvState state_ = TrainingCsvState::Idle;
};

} // namespace exo

#endif /* MASTER_TRAINING_CSV_COORDINATOR_H_ */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <exo/protocol/blepipe_proto.h>

#include <exo/protocol/master_training_csv_coordinator.h>

namespace {

struct FakeState {
    exo::SessionHeader master_header{};
    exo::Bno85Sample master_bno[9]{};
    exo::Icm45686Sample master_icm[9]{};
    unsigned master_reads = 0U;
    uint32_t largest_master_read = 0U;
    char csv[16384]{};
    size_t csv_size = 0U;
    unsigned sync_calls = 0U;
    unsigned close_calls = 0U;
    FRESULT close_result = FR_OK;
} g_fake;

struct StageFsState {
    uint8_t bytes[8192]{};
    uint32_t size = 0U;
    uint32_t cursor = 0U;
    bool open = false;
    bool unlinked = false;
    FRESULT close_result = FR_OK;
    UINT largest_read = 0U;
    unsigned read_calls = 0U;
} g_stage;

struct ReliableTxState {
    exo::RecordReliableType type[64]{};
    uint32_t next_chunk[64]{};
    uint8_t credit[64]{};
    unsigned count = 0U;
    bool succeed = true;
} g_reliable_tx;

bool fake_reliable_send(void *, uint8_t, const uint8_t *frame, uint16_t length)
{
    if (frame == nullptr || length < sizeof(exo::RecordReliableFrameHeader)) {
        return false;
    }
    exo::RecordReliableFrameHeader header{};
    memcpy(&header, frame, sizeof(header));
    const unsigned index = g_reliable_tx.count;
    if (index < 64U) {
        g_reliable_tx.type[index] =
                static_cast<exo::RecordReliableType>(header.frame_type);
        g_reliable_tx.next_chunk[index] = header.chunk_index;
        if (header.frame_type == static_cast<uint8_t>(exo::RecordReliableType::AckWindow) &&
                header.payload_len >= sizeof(exo::RecordReliableAckWindowPayload) &&
                length >= sizeof(header) + sizeof(exo::RecordReliableAckWindowPayload)) {
            exo::RecordReliableAckWindowPayload ack{};
            memcpy(&ack, frame + sizeof(header), sizeof(ack));
            g_reliable_tx.next_chunk[index] = ack.next_chunk_index;
            g_reliable_tx.credit[index] = ack.credit;
        }
    }
    ++g_reliable_tx.count;
    return g_reliable_tx.succeed;
}

void reset_reliable_tx()
{
    g_reliable_tx = ReliableTxState{};
}

uint16_t make_chunk_frame(uint8_t *frame, size_t capacity, uint8_t node_id,
        uint32_t session_id, uint32_t chunk_index, const uint8_t *payload,
        uint16_t payload_len, bool final_chunk = false)
{
    if (frame == nullptr || payload == nullptr ||
            capacity < sizeof(exo::RecordReliableFrameHeader) + payload_len) {
        return 0U;
    }
    exo::RecordReliableFrameHeader header{};
    header.command = exo::RecordCommand::ReliableFrame;
    header.proto_version = exo::kRecordReliableProtoVersion;
    header.magic = exo::kRecordReliableMagic;
    header.frame_type = static_cast<uint8_t>(exo::RecordReliableType::Chunk);
    header.source_id = node_id;
    header.session_id = session_id;
    header.chunk_index = chunk_index;
    header.byte_offset = chunk_index * exo::kRecordReliableDefaultChunkSize;
    header.payload_len = payload_len;
    header.payload_crc16 = exo::MasterNodeReliableControl::crc16_ccitt(payload, payload_len);
    header.flags = final_chunk ? static_cast<uint16_t>(exo::kRecordFlagFinalChunk) : 0U;
    memcpy(frame, &header, sizeof(header));
    memcpy(frame + sizeof(header), payload, payload_len);
    return static_cast<uint16_t>(sizeof(header) + payload_len);
}

bool fake_master_ready(const exo::MasterSdSessionRecorder &) { return true; }

const exo::SessionHeader &fake_master_header(const exo::MasterSdSessionRecorder &)
{
    return g_fake.master_header;
}

bool fake_master_read(exo::MasterSdSessionRecorder &, uint32_t offset, void *output,
        uint32_t size)
{
    ++g_fake.master_reads;
    if (size > g_fake.largest_master_read) {
        g_fake.largest_master_read = size;
    }
    const uint32_t bno_start = sizeof(exo::SessionHeader);
    const uint32_t icm_start = bno_start + g_fake.master_header.bno85_payload_size;
    if (offset >= bno_start && offset + size <= icm_start) {
        memcpy(output, reinterpret_cast<const uint8_t *>(g_fake.master_bno) +
                (offset - bno_start), size);
        return true;
    }
    const uint32_t total = icm_start + g_fake.master_header.icm45686_payload_size;
    if (offset >= icm_start && offset + size <= total) {
        memcpy(output, reinterpret_cast<const uint8_t *>(g_fake.master_icm) +
                (offset - icm_start), size);
        return true;
    }
    return false;
}

FRESULT fake_mkdir(const TCHAR *) { return FR_EXIST; }
FRESULT fake_stat(const TCHAR *, FILINFO *) { return FR_NO_FILE; }
FRESULT fake_open(FIL *, const TCHAR *, BYTE) { return FR_OK; }
FRESULT fake_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    if (g_fake.csv_size + bytes >= sizeof(g_fake.csv)) {
        *written = 0U;
        return FR_DISK_ERR;
    }
    memcpy(&g_fake.csv[g_fake.csv_size], data, bytes);
    g_fake.csv_size += bytes;
    g_fake.csv[g_fake.csv_size] = '\0';
    *written = bytes;
    return FR_OK;
}
FRESULT fake_sync(FIL *) { ++g_fake.sync_calls; return FR_OK; }
FRESULT fake_close(FIL *) { ++g_fake.close_calls; return g_fake.close_result; }
FRESULT fake_rename(const TCHAR *, const TCHAR *) { return FR_OK; }

const exo::training_csv::TrainingCsvFatFsOps kLoggerOps = {
    fake_mkdir, fake_stat, fake_open, fake_write, fake_sync, fake_close, fake_rename
};

FRESULT stage_open(FIL *, const TCHAR *, BYTE mode)
{
    g_stage.open = true;
    g_stage.cursor = 0U;
    if ((mode & FA_CREATE_ALWAYS) != 0U) {
        g_stage.size = 0U;
        g_stage.unlinked = false;
    }
    return FR_OK;
}
FRESULT stage_write(FIL *, const void *data, UINT bytes, UINT *written)
{
    memcpy(&g_stage.bytes[g_stage.cursor], data, bytes);
    g_stage.cursor += bytes;
    if (g_stage.cursor > g_stage.size) g_stage.size = g_stage.cursor;
    *written = bytes;
    return FR_OK;
}
FRESULT stage_read(FIL *, void *data, UINT bytes, UINT *received)
{
	++g_stage.read_calls;
	if (bytes > g_stage.largest_read) g_stage.largest_read = bytes;
    const uint32_t available = g_stage.size - g_stage.cursor;
    const UINT count = bytes < available ? bytes : static_cast<UINT>(available);
    memcpy(data, &g_stage.bytes[g_stage.cursor], count);
    g_stage.cursor += count;
    *received = count;
    return FR_OK;
}
FRESULT stage_seek(FIL *, FSIZE_t offset)
{
    g_stage.cursor = static_cast<uint32_t>(offset);
    return g_stage.cursor <= sizeof(g_stage.bytes) ? FR_OK : FR_DISK_ERR;
}
FRESULT stage_close(FIL *)
{
    if (g_stage.close_result == FR_OK) g_stage.open = false;
    return g_stage.close_result;
}
FRESULT stage_unlink(const TCHAR *)
{
    g_stage.unlinked = true;
    g_stage.size = 0U;
    return FR_OK;
}

const exo::node_session_staging::NodeSessionFatFsOps kStagerOps = {
    stage_open, stage_write, stage_read, stage_seek, stage_close, stage_unlink
};

} // namespace

static_assert(exo::MasterTrainingCsvCoordinator::kRowsPerService == 8U,
        "coordinator work must remain bounded");
static_assert(exo::MasterTrainingCsvCoordinator::kValidationBytesPerService <= 1024U,
        "staged CRC validation must have a fixed per-service byte budget");
static_assert(exo::MasterTrainingCsvCoordinator::kPendingChunkDepth == 24U,
        "receiver queue must hold the full default credit window");

int main()
{
    exo::training_csv_coordinator::MasterRecordingOps master_ops = {
        fake_master_ready, fake_master_header, fake_master_read
    };
    exo::MasterTrainingCsvCoordinator coordinator(&kLoggerOps, &kStagerOps, &master_ops);
    exo::MasterSdSessionRecorder recorder;

    /* Runtime flow-control requests are sanitized as one effective contract:
     * credit always fits the queue and the partial-ACK threshold stays below it. */
    if (coordinator.receiver_credit() != 24U ||
            coordinator.ack_chunk_threshold() != 8U ||
            coordinator.ack_timeout_ms() != 350U) {
        return 23;
    }
    coordinator.set_receiver_credit(1U);
    if (coordinator.receiver_credit() != 2U ||
            coordinator.ack_chunk_threshold() != 1U) {
        return 24;
    }
    coordinator.set_receiver_credit(255U);
    coordinator.set_ack_chunk_threshold(255U);
    coordinator.set_ack_timeout_ms(1U);
    if (coordinator.receiver_credit() != 24U ||
            coordinator.ack_chunk_threshold() != 23U ||
            coordinator.ack_timeout_ms() != 100U) {
        return 25;
    }
    coordinator.set_receiver_credit(0U);
    coordinator.set_ack_chunk_threshold(0U);
    coordinator.set_ack_timeout_ms(0U);
    if (coordinator.receiver_credit() != 24U ||
            coordinator.ack_chunk_threshold() != 8U ||
            coordinator.ack_timeout_ms() != 350U) {
        return 26;
    }

    /* Fill the complete advertised window without running the superloop. The
     * 25th frame must be backpressured without committing or touching SD. */
    g_fake.master_header = exo::SessionHeader{};
    g_fake.master_header.session_id = 700U;
    g_stage = StageFsState{};
    reset_reliable_tx();
    exo::MasterTrainingCsvCoordinator queue_guard(&kLoggerOps, &kStagerOps,
            &master_ops, fake_reliable_send, nullptr);
    if (!queue_guard.begin_binary_session(700U, 0x03U, 7U)) return 27;
    queue_guard.on_master_finalized(recorder);
    exo::RecordDoneMessage queue_done{};
    queue_done.command = exo::RecordCommand::RecordDone;
    queue_done.node_id = 1U;
    queue_done.session_id = 700U;
    queue_done.total_size = 25U * exo::kRecordReliableDefaultChunkSize;
    queue_guard.on_node_record_done(queue_done);
    if (!queue_guard.owns_node_link(1U) || queue_guard.owns_node_link(2U)) return 28;
    uint8_t chunk_payload[exo::kRecordReliableDefaultChunkSize]{};
    uint8_t queue_frame[sizeof(exo::RecordReliableFrameHeader) +
            exo::kRecordReliableDefaultChunkSize]{};
    for (uint32_t chunk = 0U; chunk < 24U; ++chunk) {
        const uint16_t frame_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
                1U, 700U, chunk, chunk_payload, sizeof(chunk_payload));
        queue_guard.on_node_reliable_frame(1U, queue_frame, frame_len, chunk);
    }
    if (queue_guard.pending_chunk_count() != 24U ||
            queue_guard.queue_high_water() != 24U ||
            queue_guard.queue_overflow_count() != 0U ||
            queue_guard.active_staged_bytes() != 0U ||
            queue_guard.next_expected_node_chunk() != 24U) {
        return 29;
    }
    const uint16_t overflow_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
            1U, 700U, 24U, chunk_payload, sizeof(chunk_payload));
    queue_guard.on_node_reliable_frame(1U, queue_frame, overflow_len, 24U);
    if (queue_guard.pending_chunk_count() != 24U ||
            queue_guard.queue_overflow_count() != 1U ||
            queue_guard.active_staged_bytes() != 0U ||
            queue_guard.next_expected_node_chunk() != 24U) {
        return 30;
    }

    /* A normal drain moves bytes to the stager but does not ACK a partial
     * batch. The idle timeout, threshold, duplicate, gap and final paths own
     * the only immediate-control exceptions. */
    g_stage = StageFsState{};
    reset_reliable_tx();
    exo::MasterTrainingCsvCoordinator ack_policy(&kLoggerOps, &kStagerOps,
            &master_ops, fake_reliable_send, nullptr);
    if (!ack_policy.begin_binary_session(701U, 0x03U, 8U)) return 31;
    g_fake.master_header.session_id = 701U;
    ack_policy.on_master_finalized(recorder);
    queue_done.session_id = 701U;
    queue_done.total_size = 64U * exo::kRecordReliableDefaultChunkSize;
    ack_policy.on_node_record_done(queue_done);
    ack_policy.service_reliable_control(0U); /* manifest defer */
    ack_policy.service_reliable_control(1U); /* ManifestAck */
    reset_reliable_tx();
    for (uint32_t chunk = 0U; chunk < 7U; ++chunk) {
        const uint16_t frame_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
                1U, 701U, chunk, chunk_payload, sizeof(chunk_payload));
        ack_policy.on_node_reliable_frame(1U, queue_frame, frame_len, 100U + chunk);
    }
    ack_policy.service(recorder, 107U);
    if (g_reliable_tx.count != 0U || ack_policy.ack_attempt_count() != 0U) return 32;
    const uint16_t threshold_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
            1U, 701U, 7U, chunk_payload, sizeof(chunk_payload));
    ack_policy.on_node_reliable_frame(1U, queue_frame, threshold_len, 108U);
    ack_policy.service_reliable_control(109U);
    if (g_reliable_tx.count != 1U ||
            g_reliable_tx.type[0] != exo::RecordReliableType::AckWindow ||
            g_reliable_tx.next_chunk[0] != 8U || g_reliable_tx.credit[0] > 16U ||
            ack_policy.ack_attempt_count() != 1U ||
            ack_policy.ack_success_count() != 1U ||
            !ack_policy.last_ack_status()) {
        return 33;
    }
    reset_reliable_tx();
    const uint16_t partial_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
            1U, 701U, 8U, chunk_payload, sizeof(chunk_payload));
    ack_policy.on_node_reliable_frame(1U, queue_frame, partial_len, 200U);
    ack_policy.service(recorder, 200U); /* drains one; must not ACK */
    ack_policy.service_reliable_control(549U);
    if (g_reliable_tx.count != 0U) return 34;
    ack_policy.service_reliable_control(550U);
    if (g_reliable_tx.count != 1U ||
            g_reliable_tx.type[0] != exo::RecordReliableType::AckWindow ||
            g_reliable_tx.next_chunk[0] != 9U || g_reliable_tx.credit[0] > 24U) {
        return 35;
    }
    reset_reliable_tx();
    ack_policy.on_node_reliable_frame(1U, queue_frame, partial_len, 551U);
    ack_policy.service_reliable_control(552U);
    if (g_reliable_tx.count != 1U ||
            g_reliable_tx.type[0] != exo::RecordReliableType::AckWindow) {
        return 36;
    }
    reset_reliable_tx();
    const uint16_t gap_len = make_chunk_frame(queue_frame, sizeof(queue_frame),
            1U, 701U, 10U, chunk_payload, sizeof(chunk_payload));
    ack_policy.on_node_reliable_frame(1U, queue_frame, gap_len, 553U);
    ack_policy.service_reliable_control(554U);
    if (g_reliable_tx.count != 1U ||
            g_reliable_tx.type[0] != exo::RecordReliableType::NackRange) {
        return 37;
    }

    g_stage = StageFsState{};
    reset_reliable_tx();
    exo::MasterTrainingCsvCoordinator final_policy(&kLoggerOps, &kStagerOps,
            &master_ops, fake_reliable_send, nullptr);
    if (!final_policy.begin_binary_session(702U, 0x03U, 9U)) return 38;
    g_fake.master_header.session_id = 702U;
    final_policy.on_master_finalized(recorder);
    queue_done.session_id = 702U;
    exo::SessionHeader final_payload{};
    final_payload.magic = exo::kSessionMagic;
    final_payload.version = exo::kSessionFormatVersion;
    final_payload.node_id = 1U;
    final_payload.session_id = 702U;
    final_payload.completion_flag = exo::kSessionComplete;
    final_payload.header_crc32 = exo::session_header_crc(final_payload);
    queue_done.total_size = sizeof(final_payload);
    queue_done.payload_crc32 = final_payload.payload_crc32;
    final_policy.on_node_record_done(queue_done);
    final_policy.service_reliable_control(0U);
    final_policy.service_reliable_control(1U);
    reset_reliable_tx();
    uint8_t final_frame[sizeof(exo::RecordReliableFrameHeader) + sizeof(final_payload)]{};
    const uint16_t final_len = make_chunk_frame(final_frame, sizeof(final_frame),
            1U, 702U, 0U, reinterpret_cast<const uint8_t *>(&final_payload),
            sizeof(final_payload), true);
    final_policy.on_node_reliable_frame(1U, final_frame, final_len, 10U);
    final_policy.service_reliable_control(11U);
    if (g_reliable_tx.count != 1U ||
            g_reliable_tx.type[0] != exo::RecordReliableType::AckWindow ||
            final_policy.state() != exo::TrainingCsvState::ReceiveNode) {
        return 39;
    }
    final_policy.service(recorder, 12U);
    if (final_policy.state() != exo::TrainingCsvState::ValidateNode) return 40;

    final_policy.note_suppressed_relay();
    final_policy.note_sd_flush_duration_ms(7U);
    final_policy.note_sd_flush_duration_ms(3U);
    if (final_policy.received_chunk_count() != 1U ||
            final_policy.suppressed_relay_count() != 1U ||
            final_policy.sd_flush_count() != 2U ||
            final_policy.sd_flush_max_duration_ms() != 7U) {
        return 41;
    }

    g_fake.master_header.magic = exo::kSessionMagic;
    g_fake.master_header.version = exo::kSessionFormatVersion;
    g_fake.master_header.node_id = 0U;
    g_fake.master_header.session_id = 99U;
    g_fake.master_header.sensor_mask = 0x03U;
    g_fake.master_header.completion_flag = exo::kSessionComplete;
    g_fake.master_header.bno85_sample_count = 9U;
    g_fake.master_header.icm45686_sample_count = 9U;
    g_fake.master_header.bno85_attempted_count = 9U;
    g_fake.master_header.icm45686_attempted_count = 9U;
    g_fake.master_header.bno85_captured_count = 9U;
    g_fake.master_header.icm45686_captured_count = 9U;
    g_fake.master_header.bno85_payload_size = sizeof(g_fake.master_bno);
    g_fake.master_header.icm45686_payload_size = sizeof(g_fake.master_icm);
    for (uint32_t index = 0U; index < 9U; ++index) {
        g_fake.master_bno[index].offset_us = 100U + index;
        g_fake.master_icm[index].accel_x = static_cast<int16_t>(index);
    }

    if (!coordinator.begin_session(99U, 0x1FU) ||
            coordinator.state() != exo::TrainingCsvState::WaitingForMaster ||
            coordinator.active_session_id() != 99U ||
            coordinator.expected_source_mask() != 0x1FU) {
        return 1;
    }
    for (uint32_t index = 0U; index < 9U; ++index) {
        if (!coordinator.note_master_icm_time(99U, 1000U + index)) {
            return 2;
        }
    }
    if (coordinator.note_master_icm_time(98U, 5678U) ||
            coordinator.ledger_count() != 9U) {
        return 2;
    }

    exo::MasterTrainingCsvCoordinator cancelled(&kLoggerOps, &kStagerOps, &master_ops);
    if (!cancelled.begin_session(123U, 0x01U) ||
            !cancelled.note_master_icm_time(123U, 42U) ||
            !cancelled.cancel_session() ||
            cancelled.state() != exo::TrainingCsvState::Idle ||
            cancelled.active_session_id() != 0U || cancelled.ledger_count() != 0U) {
        return 21;
    }

    /* A mismatched RecordDone callback must not start or replace staging. */
    exo::RecordDoneMessage done{};
    done.command = exo::RecordCommand::RecordDone;
    done.node_id = 1U;
    done.session_id = 98U;
    done.total_size = sizeof(exo::SessionHeader);
    coordinator.on_node_record_done(done);
    if (coordinator.state() != exo::TrainingCsvState::WaitingForMaster ||
            coordinator.active_node_id() != 0U) {
        return 3;
    }

    /* Invalid and CRC-corrupt reliable frames are observer-only failures. */
    coordinator.on_master_finalized(recorder);
    if (coordinator.state() != exo::TrainingCsvState::ConvertMasterBno ||
            !coordinator.master_ledger_matches()) {
        return 4;
    }
    coordinator.service(recorder, 1U);
    if (coordinator.master_bno_index() != 8U || g_fake.master_reads != 8U) {
        return 5;
    }
    coordinator.service(recorder, 2U);
    if (coordinator.master_bno_index() != 9U ||
            coordinator.state() != exo::TrainingCsvState::ConvertMasterIcm) {
        return 6;
    }
    coordinator.service(recorder, 3U);
    if (coordinator.master_icm_index() != 8U) {
        return 7;
    }
    coordinator.service(recorder, 4U);
    if (coordinator.master_icm_index() != 9U ||
            coordinator.completed_source_mask() != 0x01U ||
            coordinator.state() != exo::TrainingCsvState::WaitingForNode ||
            strstr(g_fake.csv, ",MASTER,BNO85,0,100,") == nullptr ||
            strstr(g_fake.csv, ",MASTER,ICM45686,8,1008,") == nullptr) {
        return 8;
    }

    uint8_t bad_frame[sizeof(exo::RecordReliableFrameHeader) + 1U]{};
    exo::RecordReliableFrameHeader header{};
    header.command = exo::RecordCommand::ReliableFrame;
    header.proto_version = exo::kRecordReliableProtoVersion;
    header.magic = exo::kRecordReliableMagic;
    header.frame_type = static_cast<uint8_t>(exo::RecordReliableType::Chunk);
    header.source_id = 1U;
    header.session_id = 99U;
    header.payload_len = 1U;
    header.payload_crc16 = 0U;
    memcpy(bad_frame, &header, sizeof(header));
    bad_frame[sizeof(header)] = 0xA5U;
    coordinator.on_node_reliable_frame(1U, bad_frame, sizeof(bad_frame));
    if (coordinator.completed_source_mask() != 0x01U ||
            coordinator.state() != exo::TrainingCsvState::WaitingForNode) {
        return 9;
    }

    /* A complete Node session validates, preserves BNO time, and derives 10 ms ICM time. */
    struct NodeImage {
        exo::SessionHeader header;
        exo::Bno85Sample bno;
        exo::Icm45686Sample icm[20];
    } image{};
    image.header.magic = exo::kSessionMagic;
    image.header.version = exo::kSessionFormatVersion;
    image.header.node_id = 1U;
    image.header.session_id = 99U;
    image.header.completion_flag = exo::kSessionComplete;
    image.header.bno85_sample_count = 1U;
    image.header.icm45686_sample_count = 20U;
    image.header.bno85_attempted_count = 1U;
    image.header.icm45686_attempted_count = 20U;
    image.header.bno85_captured_count = 1U;
    image.header.icm45686_captured_count = 20U;
    image.header.bno85_payload_size = sizeof(image.bno);
    image.header.icm45686_payload_size = sizeof(image.icm);
    image.bno.offset_us = 54321U;
    image.icm[7].gyro_z = 77;
    image.header.payload_crc32 = exo::crc32(&image.bno,
            sizeof(image) - sizeof(image.header));
    image.header.header_crc32 = exo::session_header_crc(image.header);

    done.session_id = 99U;
    done.total_size = sizeof(image);
    done.payload_crc32 = image.header.payload_crc32;
    coordinator.on_node_record_done(done);
    if (coordinator.state() != exo::TrainingCsvState::ReceiveNode) return 10;

    uint8_t frame[sizeof(exo::RecordReliableFrameHeader) +
            exo::kRecordReliableDefaultChunkSize]{};
    const uint8_t *image_bytes = reinterpret_cast<const uint8_t *>(&image);
    uint32_t image_offset = 0U;
    uint32_t image_chunk = 0U;
    while (image_offset < sizeof(image)) {
        const uint32_t remaining = static_cast<uint32_t>(sizeof(image)) - image_offset;
        const uint16_t take = static_cast<uint16_t>(remaining <
                exo::kRecordReliableDefaultChunkSize ? remaining :
                exo::kRecordReliableDefaultChunkSize);
        const bool final = image_offset + take == sizeof(image);
        const uint16_t frame_len = make_chunk_frame(frame, sizeof(frame), 1U, 99U,
                image_chunk, image_bytes + image_offset, take, final);
        coordinator.on_node_reliable_frame(1U, frame, frame_len, 5U);
        image_offset += take;
        ++image_chunk;
    }
    if (coordinator.state() != exo::TrainingCsvState::ReceiveNode) return 11;
	g_stage.largest_read = 0U;
	const unsigned reads_before_validation = g_stage.read_calls;
    coordinator.service(recorder, 5U);
	if (coordinator.state() != exo::TrainingCsvState::ValidateNode ||
			g_stage.read_calls != reads_before_validation + 1U ||
			g_stage.largest_read > exo::MasterTrainingCsvCoordinator::kValidationBytesPerService) {
		return 22;
	}
    coordinator.service(recorder, 6U);
    coordinator.service(recorder, 7U);
	coordinator.service(recorder, 8U);
	coordinator.service(recorder, 9U);
	coordinator.service(recorder, 10U);
	coordinator.service(recorder, 11U);
	coordinator.service(recorder, 12U);
    if (coordinator.completed_source_mask() != 0x03U || g_stage.unlinked ||
            strstr(g_fake.csv, ",NODE1,BNO85,0,54321,") == nullptr ||
            strstr(g_fake.csv, ",NODE1,ICM45686,7,70000,") == nullptr) {
        return 12;
    }

    coordinator.shutdown(10U);
    if (coordinator.state() != exo::TrainingCsvState::Idle || g_fake.close_calls != 1U) {
        return 13;
    }

    exo::MasterTrainingCsvCoordinator logger_close_retry(&kLoggerOps, &kStagerOps,
            &master_ops);
    if (!logger_close_retry.begin_session(100U, 0x01U)) return 14;
    g_fake.master_header.session_id = 100U;
    logger_close_retry.on_master_finalized(recorder);
    g_fake.close_result = FR_DISK_ERR;
    logger_close_retry.shutdown(11U);
    if (logger_close_retry.state() != exo::TrainingCsvState::CsvError ||
            logger_close_retry.failure_site() != exo::TrainingFailSite::Site9 ||
            logger_close_retry.failure_node_id() != 0U ||
            logger_close_retry.failure_csv_operation() !=
                    exo::training_csv::TrainingCsvLogOperation::Close ||
            logger_close_retry.failure_csv_result() != FR_DISK_ERR ||
            logger_close_retry.begin_session(101U, 0x01U)) return 15;
    g_fake.close_result = FR_OK;
    logger_close_retry.shutdown(12U);
    if (logger_close_retry.state() != exo::TrainingCsvState::Idle ||
            !logger_close_retry.begin_session(101U, 0x01U)) return 16;
    logger_close_retry.shutdown(13U);

    g_fake.master_header = exo::SessionHeader{};
    g_fake.master_header.session_id = 102U;
    exo::MasterTrainingCsvCoordinator stage_close_retry(&kLoggerOps, &kStagerOps,
            &master_ops);
    if (!stage_close_retry.begin_session(102U, 0x03U)) return 17;
    stage_close_retry.on_master_finalized(recorder);
    stage_close_retry.service(recorder, 14U);
    stage_close_retry.service(recorder, 15U);
    done.session_id = 102U;
    stage_close_retry.on_node_record_done(done);
    if (stage_close_retry.state() != exo::TrainingCsvState::ReceiveNode) return 18;
    g_stage.close_result = FR_DISK_ERR;
    stage_close_retry.shutdown(16U);
    if (stage_close_retry.state() != exo::TrainingCsvState::StageError ||
            stage_close_retry.failure_site() != exo::TrainingFailSite::Site10 ||
            stage_close_retry.failure_node_id() != 1U ||
            stage_close_retry.failure_stager_operation() !=
                    exo::node_session_staging::NodeSessionStageOperation::ShutdownClose ||
            stage_close_retry.failure_stager_result() != FR_DISK_ERR ||
            stage_close_retry.begin_session(103U, 0x01U)) return 19;
    g_stage.close_result = FR_OK;
    stage_close_retry.shutdown(17U);
    if (stage_close_retry.state() != exo::TrainingCsvState::Idle ||
            !stage_close_retry.begin_session(103U, 0x01U)) return 20;
    stage_close_retry.shutdown(18U);
    (void)recorder;
    return 0;
}

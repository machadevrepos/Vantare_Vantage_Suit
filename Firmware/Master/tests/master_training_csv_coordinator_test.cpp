#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "blepipe_proto.h"

#include "MASTER_TRAINING_CSV_COORDINATOR.h"

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
    uint8_t bytes[512]{};
    uint32_t size = 0U;
    uint32_t cursor = 0U;
    bool open = false;
    bool unlinked = false;
    FRESULT close_result = FR_OK;
    UINT largest_read = 0U;
    unsigned read_calls = 0U;
} g_stage;

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
static_assert(exo::MasterTrainingCsvCoordinator::kValidationBytesPerService == 256U,
        "staged CRC validation must have a fixed per-service byte budget");

int main()
{
    exo::training_csv_coordinator::MasterRecordingOps master_ops = {
        fake_master_ready, fake_master_header, fake_master_read
    };
    exo::MasterTrainingCsvCoordinator coordinator(&kLoggerOps, &kStagerOps, &master_ops);
    exo::MasterSdSessionRecorder recorder;

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
    static_assert(sizeof(NodeImage) - sizeof(exo::SessionHeader) >
            exo::MasterTrainingCsvCoordinator::kValidationBytesPerService,
            "test session must require more than one CRC-validation service step");
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

    uint8_t frame[sizeof(exo::RecordReliableFrameHeader) + sizeof(image)]{};
    header.payload_len = sizeof(image);
    header.payload_crc16 = blepipe_crc16_ccitt(
            reinterpret_cast<const uint8_t *>(&image), sizeof(image));
    header.flags = exo::kRecordFlagFinalChunk;
    memcpy(frame, &header, sizeof(header));
    memcpy(&frame[sizeof(header)], &image, sizeof(image));
    coordinator.on_node_reliable_frame(1U, frame, sizeof(frame));
    if (coordinator.state() != exo::TrainingCsvState::ValidateNode) return 11;
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

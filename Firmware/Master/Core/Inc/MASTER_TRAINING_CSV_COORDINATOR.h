#ifndef MASTER_TRAINING_CSV_COORDINATOR_H_
#define MASTER_TRAINING_CSV_COORDINATOR_H_
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "app_fatfs.h"
#include <BLE_RECORD_PROTOCOL.h>
#include <MASTER_NODE_RELIABLE_CONTROL.h>
#include <MASTER_NODE_SESSION_STAGER.h>
#include <MASTER_NODE_TRANSFER_WINDOW.h>
#include <MASTER_SD_SESSION_RECORDER.h>
#include <MASTER_SESSION_TIMESTAMP_LEDGER.h>
#include <MASTER_TRAINING_CSV_LOGGER.h>

/* Production Master builds are binary-only.  Leave this default disabled so
 * the legacy CSV coordinator remains available to host tests and opt-in builds. */
#ifndef EXO_MASTER_BINARY_ONLY_BUILD
#define EXO_MASTER_BINARY_ONLY_BUILD 0
#endif

namespace exo {
enum class TrainingCsvState : uint8_t {
Idle, WaitingForMaster, ConvertMasterBno, ConvertMasterIcm, WaitingForNode,
ReceiveNode, ValidateNode, ConvertNodeBno, ConvertNodeIcm, Complete,
CsvError, StageError, BinaryFinalizeNode
};
enum class TrainingFailSite : uint8_t {
None = 0xFFU,
Site0 = 0, Site1, Site2, Site3, Site4, Site5, Site6, Site7, Site8, Site9,
Site10, Site11, Site12, Site13, Site14, Site15, Site16, Site17, Site18,
Site19, Site20, SessionStall
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
}
class MasterTrainingCsvCoordinator {
public:
static constexpr uint32_t kRowsPerService = 8U;
static constexpr uint32_t kValidationBytesPerService = 256U;
static constexpr uint32_t kSessionStallMs = 30000U;
/* A node that goes quiet mid-upload only costs that node's rows. The remaining
 * expected sources are still staged and the CSV is still published, so one flaky
 * link cannot discard an entire capture. */
static constexpr uint32_t kNodeStallMs = 30000U;
static constexpr uint8_t kNodeReceiverCredit = 8U;
/* Re-advertise the receive window when the active node stops sending, so a lost
 * ACK does not idle the link until the stall timeout expires. */
static constexpr uint32_t kNodeAckRearmMs = 300U;
explicit MasterTrainingCsvCoordinator(
const training_csv::TrainingCsvFatFsOps *logger_ops = nullptr,
const node_session_staging::NodeSessionFatFsOps *stager_ops = nullptr,
const training_csv_coordinator::MasterRecordingOps *master_ops = nullptr,
MasterNodeReliableControl::SendFn reliable_send = nullptr,
void *reliable_context = nullptr)
: logger_(logger_ops), stager_(stager_ops),
reliable_control_(reliable_send, reliable_context),
master_ops_(master_ops != nullptr ? master_ops :
&training_csv_coordinator::kDefaultMasterRecordingOps)
{
}
/* Without this the control frames fall back to exo_hub_ble_write(), which is the
 * inbound phone-write handler: every ACK/NACK would re-enter the command
 * dispatcher and be echoed back to the desktop tool. Point it straight at the
 * node link instead. */
void set_reliable_transport(MasterNodeReliableControl::SendFn send_fn, void *context)
{
reliable_control_.set_transport(send_fn, context);
}
bool begin_session(uint32_t session_id, uint8_t expected_source_mask)
{
const bool recoverable_error = partial_finalized_ &&
(state_ == TrainingCsvState::CsvError || state_ == TrainingCsvState::StageError);
if (state_ != TrainingCsvState::Idle && state_ != TrainingCsvState::Complete &&
!recoverable_error) return false;
if (logger_.has_open_file() || stager_.active()) return false;
if ((expected_source_mask & 0x01U) == 0U ||
(expected_source_mask & static_cast<uint8_t>(~0x1FU)) != 0U) return false;
clear_failure();
reliable_control_.reset();
transfer_window_.reset();
reliable_defer_services_ = 0U;
last_progress_ms_ = 0U;
progress_seq_ = 0U;
last_progress_seq_ = 0U;
partial_finalized_ = false;
#if !EXO_MASTER_BINARY_ONLY_BUILD
binary_only_ = false;
#endif
file_index_ = 0U;
cleanup_pending_mask_ = 0U;
active_session_id_ = session_id;
expected_source_mask_ = expected_source_mask;
completed_source_mask_ = 0U;
failed_source_mask_ = 0U;
active_node_id_ = 0U;
master_bno_index_ = master_icm_index_ = 0U;
node_bno_index_ = node_icm_index_ = 0U;
master_ledger_matches_ = false;
memset(&master_header_, 0, sizeof(master_header_));
ledger_.reset(session_id);
state_ = TrainingCsvState::WaitingForMaster;
return true;
}
bool begin_binary_session(uint32_t session_id, uint8_t expected_source_mask, uint16_t file_index)
{
if (file_index == 0U || file_index > 9999U) return false;
if (!begin_session(session_id, expected_source_mask)) return false;
#if !EXO_MASTER_BINARY_ONLY_BUILD
binary_only_ = true;
#endif
file_index_ = file_index;
return true;
}
bool note_master_icm_time(uint32_t session_id, uint32_t session_time_us)
{
if (binary_only_) return session_id == active_session_id_;
if (session_id != active_session_id_ ||
(state_ != TrainingCsvState::WaitingForMaster &&
state_ != TrainingCsvState::ConvertMasterBno &&
state_ != TrainingCsvState::ConvertMasterIcm)) return false;
return ledger_.append_icm(session_time_us);
}
bool cancel_session()
{
if (state_ != TrainingCsvState::WaitingForMaster || logger_.has_open_file() ||
stager_.active()) return false;
active_session_id_ = 0U;
expected_source_mask_ = completed_source_mask_ = active_node_id_ = 0U;
failed_source_mask_ = 0U;
master_bno_index_ = master_icm_index_ = node_bno_index_ = node_icm_index_ = 0U;
master_ledger_matches_ = false;
memset(&master_header_, 0, sizeof(master_header_));
ledger_.reset(0U);
reliable_control_.reset();
transfer_window_.reset();
reliable_defer_services_ = 0U;
last_progress_ms_ = progress_seq_ = last_progress_seq_ = 0U;
partial_finalized_ = false;
#if !EXO_MASTER_BINARY_ONLY_BUILD
binary_only_ = false;
#endif
file_index_ = 0U;
cleanup_pending_mask_ = 0U;
state_ = TrainingCsvState::Idle;
return true;
}
void on_master_finalized(MasterSdSessionRecorder &recorder)
{
if (state_ != TrainingCsvState::WaitingForMaster || !master_ops_->ready_fn(recorder) ||
master_ops_->header_fn(recorder).session_id != active_session_id_) return;
master_header_ = master_ops_->header_fn(recorder);
if (master_header_.bno85_sample_count != master_header_.bno85_captured_count ||
master_header_.icm45686_sample_count != master_header_.icm45686_captured_count) {
fail(binary_only_ ? TrainingCsvState::StageError : TrainingCsvState::CsvError, TrainingFailSite::Site0);
return;
}
if (binary_only_) {
completed_source_mask_ = static_cast<uint8_t>(completed_source_mask_ | 0x01U);
state_ = ((completed_source_mask_ | failed_source_mask_) & expected_source_mask_) == expected_source_mask_ ?
TrainingCsvState::Complete : TrainingCsvState::WaitingForNode;
return;
}
master_ledger_matches_ = !ledger_.overflowed() &&
ledger_.session_id() == active_session_id_ &&
ledger_.icm_count() == master_header_.icm45686_sample_count;
if (!logger_.begin(active_session_id_, expected_source_mask_, 0U) ||
!logger_.set_source_metadata(0U, master_header_)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site1);
return;
}
state_ = TrainingCsvState::ConvertMasterBno;
}
void on_node_record_done(const RecordDoneMessage &done)
{
if (partial_finalized_ || state_ != TrainingCsvState::WaitingForNode ||
done.command != RecordCommand::RecordDone ||
done.session_id != active_session_id_ || done.node_id < 1U || done.node_id > 4U ||
(expected_source_mask_ & static_cast<uint8_t>(1U << done.node_id)) == 0U ||
(completed_source_mask_ & static_cast<uint8_t>(1U << done.node_id)) != 0U ||
(failed_source_mask_ & static_cast<uint8_t>(1U << done.node_id)) != 0U) return;
if (!stager_.begin(done, binary_only_ ? file_index_ : logger_.file_index())) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site2);
return;
}
if (!transfer_window_.begin(static_cast<uint8_t>(done.node_id), done.session_id,
done.total_size, kRecordReliableDefaultChunkSize) ||
!reliable_control_.begin(done, kRecordReliableDefaultChunkSize,
kNodeReceiverCredit)) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site2);
return;
}
reliable_defer_services_ = 1U;
active_node_id_ = static_cast<uint8_t>(done.node_id);
node_bno_index_ = node_icm_index_ = 0U;
state_ = TrainingCsvState::ReceiveNode;
}
void on_node_reliable_frame(uint8_t node_id, const uint8_t *frame, uint16_t length)
{
if (partial_finalized_ || frame == nullptr ||
length < sizeof(RecordReliableFrameHeader) || node_id < 1U || node_id > 4U) return;
RecordReliableFrameHeader header{};
memcpy(&header, frame, sizeof(header));
if (header.command != RecordCommand::ReliableFrame ||
header.proto_version != kRecordReliableProtoVersion ||
header.magic != kRecordReliableMagic ||
header.frame_type != static_cast<uint8_t>(RecordReliableType::Chunk) ||
header.source_id != node_id || header.session_id != active_session_id_ ||
static_cast<size_t>(sizeof(header)) + header.payload_len != length ||
state_ != TrainingCsvState::ReceiveNode || node_id != active_node_id_) return;
const uint8_t *payload = frame + sizeof(header);
const bool crc_ok = MasterNodeReliableControl::crc16_ccitt(payload,
header.payload_len) == header.payload_crc16;
const bool final_chunk = (header.flags & kRecordFlagFinalChunk) != 0U;
const NodeTransferInspection inspection = transfer_window_.inspect(node_id,
header.session_id, header.chunk_index, header.byte_offset,
header.payload_len, crc_ok, final_chunk);
switch (inspection.decision) {
case NodeTransferDecision::Ignore:
return;
case NodeTransferDecision::Duplicate:
(void)reliable_control_.ack_window(transfer_window_.next_chunk(),
kNodeReceiverCredit);
return;
case NodeTransferDecision::NackGap:
(void)reliable_control_.nack_range(inspection.request_chunk, 1U);
return;
case NodeTransferDecision::NackCorrupt:
(void)reliable_control_.nack_range(inspection.request_chunk, 1U,
kRecordFlagCrcMismatch);
return;
case NodeTransferDecision::Accept:
case NodeTransferDecision::Complete:
break;
}
if (!stager_.accept_chunk(node_id, header.session_id, header.byte_offset,
payload, header.payload_len) || !transfer_window_.commit(inspection)) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site3);
return;
}
++progress_seq_;
(void)reliable_control_.ack_window(inspection.next_chunk, kNodeReceiverCredit);
if (inspection.decision == NodeTransferDecision::Complete) {
state_ = TrainingCsvState::ValidateNode;
}
}
/* Flush queued reliable-control frames (ACK windows, NACKs) without the
full state-machine pass. Called straight after BLE event dispatch so an
ACK reaches the node within the iteration it was queued instead of after
all remaining superloop work; honors reliable_defer_services_ so a
freshly queued ManifestAck still transmits before any ACK. */
void service_reliable_control(uint32_t now_ms)
{
if (reliable_defer_services_ != 0U) {
--reliable_defer_services_;
return;
}
(void)reliable_control_.service(now_ms);
}
void service(MasterSdSessionRecorder &recorder, uint32_t now_ms)
{
if (reliable_defer_services_ != 0U) {
--reliable_defer_services_;
} else {
(void)reliable_control_.service(now_ms);
}
const TrainingCsvState entry_state = state_;
service_state(recorder, now_ms);
if (state_ != entry_state) ++progress_seq_;
if (progress_seq_ != last_progress_seq_ || last_progress_ms_ == 0U) {
last_progress_seq_ = progress_seq_;
last_progress_ms_ = now_ms;
}
service_finalize(now_ms);
}
void finalize_partial(uint32_t now_ms)
{
if (partial_finalized_) return;
partial_finalized_ = true;
if (!binary_only_) (void)logger_.shutdown(now_ms);
/* A failed stage must not leave a truncated R####N#.BIN that consumes the run
 * index forever; the node's flash copy is the retry source. */
(void)stager_.abandon_and_unlink();
transfer_window_.reset();
reliable_control_.reset();
active_node_id_ = 0U;
}
bool partial_finalized() const { return partial_finalized_; }
void service_state(MasterSdSessionRecorder &recorder, uint32_t now_ms)
{
#if EXO_MASTER_BINARY_ONLY_BUILD
(void)recorder;
#endif
switch (state_) {
#if !EXO_MASTER_BINARY_ONLY_BUILD
case TrainingCsvState::ConvertMasterBno:
service_master_bno(recorder, now_ms); break;
case TrainingCsvState::ConvertMasterIcm:
service_master_icm(recorder, now_ms); break;
#endif
case TrainingCsvState::ValidateNode:
switch (stager_.validation_status()) {
case node_session_staging::NodeSessionValidationStatus::Idle:
if (!stager_.begin_validation())
fail(TrainingCsvState::StageError, TrainingFailSite::Site4);
break;
case node_session_staging::NodeSessionValidationStatus::InProgress:
if (!stager_.step_validation(kValidationBytesPerService))
fail(TrainingCsvState::StageError, TrainingFailSite::Site5);
break;
case node_session_staging::NodeSessionValidationStatus::ReadyToFinalize:
if (stager_.finalize_validation()) {
if (binary_only_) {
begin_binary_node_finalize();
break;
}
if (!logger_.set_source_metadata(active_node_id_, stager_.header())) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site6);
break;
}
(void)reliable_control_.verify_ok(stager_.header().payload_crc32);
state_ = TrainingCsvState::ConvertNodeBno;
} else fail(TrainingCsvState::StageError, TrainingFailSite::Site6);
break;
case node_session_staging::NodeSessionValidationStatus::Complete:
if (binary_only_) {
begin_binary_node_finalize();
break;
}
if (!logger_.set_source_metadata(active_node_id_, stager_.header())) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site6);
break;
}
(void)reliable_control_.verify_ok(stager_.header().payload_crc32);
state_ = TrainingCsvState::ConvertNodeBno;
break;
case node_session_staging::NodeSessionValidationStatus::Failed:
fail(TrainingCsvState::StageError, TrainingFailSite::Site7); break;
}
break;
#if !EXO_MASTER_BINARY_ONLY_BUILD
case TrainingCsvState::ConvertNodeBno:
service_node_bno(now_ms); break;
case TrainingCsvState::ConvertNodeIcm:
service_node_icm(now_ms); break;
#endif
case TrainingCsvState::BinaryFinalizeNode:
if (!reliable_control_.pending()) complete_binary_node(now_ms);
break;
case TrainingCsvState::WaitingForMaster:
case TrainingCsvState::WaitingForNode:
case TrainingCsvState::ReceiveNode:
#if !EXO_MASTER_BINARY_ONLY_BUILD
if (!binary_only_ && logger_.ready() && !logger_.service(now_ms))
fail(TrainingCsvState::CsvError, TrainingFailSite::Site8);
#endif
break;
default:
break;
}
}
void service_finalize(uint32_t now_ms)
{
switch (state_) {
case TrainingCsvState::CsvError:
case TrainingCsvState::StageError:
finalize_partial(now_ms); break;
case TrainingCsvState::ReceiveNode:
if ((now_ms - last_progress_ms_) >= kNodeStallMs) abandon_active_node(now_ms);
else (void)reliable_control_.rearm_ack_window(now_ms, kNodeAckRearmMs);
break;
case TrainingCsvState::BinaryFinalizeNode:
if ((now_ms - last_progress_ms_) >= kNodeStallMs) {
cleanup_pending_mask_ = static_cast<uint8_t>(cleanup_pending_mask_ | static_cast<uint8_t>(1U << active_node_id_));
reliable_control_.reset();
complete_binary_node(now_ms);
}
break;
case TrainingCsvState::WaitingForNode:
/* Nothing arrived from any remaining node in time. Publish what was
 * staged rather than discarding the sources that did complete. */
if ((now_ms - last_progress_ms_) >= kSessionStallMs) abandon_remaining_nodes(now_ms);
break;
default:
break;
}
}
void shutdown(uint32_t now_ms)
{
if (!binary_only_) (void)logger_.shutdown(now_ms);
(void)stager_.abandon_and_unlink();
reliable_control_.reset();
transfer_window_.reset();
if (!binary_only_ && logger_.has_open_file()) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site9); return;
}
if (stager_.active()) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site10); return;
}
active_node_id_ = 0U;
state_ = TrainingCsvState::Idle;
}
TrainingCsvState state() const { return state_; }
uint32_t active_session_id() const { return active_session_id_; }
uint8_t active_node_id() const { return active_node_id_; }
uint8_t expected_source_mask() const { return expected_source_mask_; }
uint8_t completed_source_mask() const { return completed_source_mask_; }
bool binary_only() const { return binary_only_; }
uint16_t file_index() const { return binary_only_ ? file_index_ : logger_.file_index(); }
uint8_t cleanup_pending_mask() const { return cleanup_pending_mask_; }
/* Expected sources that were written off after stalling. Non-zero means the
 * published CSV is missing those sources' rows. */
uint8_t failed_source_mask() const { return failed_source_mask_; }
uint32_t ledger_count() const { return ledger_.icm_count(); }
uint32_t master_bno_index() const { return master_bno_index_; }
uint32_t master_icm_index() const { return master_icm_index_; }
uint32_t node_bno_index() const { return node_bno_index_; }
uint32_t node_icm_index() const { return node_icm_index_; }
bool master_ledger_matches() const { return master_ledger_matches_; }
const MasterTrainingCsvLogger &logger() const { return logger_; }
const MasterNodeSessionStager &stager() const { return stager_; }
uint32_t next_expected_node_chunk() const { return transfer_window_.next_chunk(); }
uint32_t next_expected_node_offset() const { return transfer_window_.next_offset(); }
bool reliable_control_pending() const { return reliable_control_.pending(); }
uint32_t reliable_control_attempt_count() const { return reliable_control_.attempt_count(); }
TrainingFailSite failure_site() const { return failure_site_; }
uint8_t failure_node_id() const { return failure_node_id_; }
node_session_staging::NodeSessionStageOperation failure_stager_operation() const
{ return failure_stager_operation_; }
FRESULT failure_stager_result() const { return failure_stager_result_; }
training_csv::TrainingCsvLogOperation failure_csv_operation() const
{ return failure_csv_operation_; }
FRESULT failure_csv_result() const { return failure_csv_result_; }
void fail_durability() { fail(binary_only_ ? TrainingCsvState::StageError : TrainingCsvState::CsvError, TrainingFailSite::Site20); }
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
while (rows < kRowsPerService && master_bno_index_ < master_header_.bno85_sample_count) {
Bno85Sample sample{};
const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
master_bno_index_ * static_cast<uint32_t>(sizeof(Bno85Sample));
if (!master_ops_->read_fn(recorder, offset, &sample, sizeof(sample)) ||
!logger_.append_bno(0U, sample.offset_us, sample, false, 0U, now_ms)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site11); return;
}
++master_bno_index_; ++rows;
}
if (master_bno_index_ == master_header_.bno85_sample_count) {
if (master_ledger_matches_) state_ = TrainingCsvState::ConvertMasterIcm;
else complete_source(0U, now_ms, false);
}
}
void service_master_icm(MasterSdSessionRecorder &recorder, uint32_t now_ms)
{
uint32_t rows = 0U;
while (rows < kRowsPerService && master_icm_index_ < master_header_.icm45686_sample_count) {
Icm45686Sample sample{};
uint32_t session_time_us = 0U;
const uint32_t offset = static_cast<uint32_t>(sizeof(SessionHeader)) +
master_header_.bno85_payload_size +
master_icm_index_ * static_cast<uint32_t>(sizeof(Icm45686Sample));
if (!ledger_.icm_time(master_icm_index_, session_time_us) ||
!master_ops_->read_fn(recorder, offset, &sample, sizeof(sample)) ||
!logger_.append_icm(0U, session_time_us, sample, now_ms)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site12); return;
}
++master_icm_index_; ++rows;
}
if (master_icm_index_ == master_header_.icm45686_sample_count)
complete_source(0U, now_ms, false);
}
void service_node_bno(uint32_t now_ms)
{
uint32_t rows = 0U;
while (rows < kRowsPerService && node_bno_index_ < stager_.header().bno85_sample_count) {
Bno85Sample sample{};
if (!stager_.read_bno(node_bno_index_, sample)) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site13); return;
}
if (!logger_.append_bno(active_node_id_, sample.offset_us, sample, false, 0U, now_ms)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site14); return;
}
++node_bno_index_; ++rows;
}
if (node_bno_index_ == stager_.header().bno85_sample_count)
state_ = TrainingCsvState::ConvertNodeIcm;
}
void service_node_icm(uint32_t now_ms)
{
uint32_t rows = 0U;
while (rows < kRowsPerService && node_icm_index_ < stager_.header().icm45686_sample_count) {
Icm45686Sample sample{};
if (!stager_.read_icm(node_icm_index_, sample)) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site15); return;
}
if (!logger_.append_icm(active_node_id_, sample.offset_us, sample, now_ms)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site16); return;
}
++node_icm_index_; ++rows;
}
if (node_icm_index_ == stager_.header().icm45686_sample_count)
complete_source(active_node_id_, now_ms, true);
}
void begin_binary_node_finalize()
{
const uint8_t node_id = active_node_id_;
const uint32_t payload_crc32 = stager_.header().payload_crc32;
if (!stager_.discard_after_success()) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site18);
return;
}
if (!reliable_control_.verify_ok(payload_crc32)) {
cleanup_pending_mask_ = static_cast<uint8_t>(cleanup_pending_mask_ | static_cast<uint8_t>(1U << node_id));
}
state_ = TrainingCsvState::BinaryFinalizeNode;
}
void complete_binary_node(uint32_t now_ms)
{
if (active_node_id_ < 1U || active_node_id_ > 4U) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site18);
return;
}
completed_source_mask_ = static_cast<uint8_t>(completed_source_mask_ |
static_cast<uint8_t>(1U << active_node_id_));
active_node_id_ = 0U;
transfer_window_.reset();
reliable_control_.reset();
settle_after_source(now_ms);
}
void complete_source(uint8_t source_id, uint32_t now_ms, bool discard_stage)
{
if (!logger_.mark_source_complete(source_id, now_ms)) {
fail(TrainingCsvState::CsvError, TrainingFailSite::Site17); return;
}
completed_source_mask_ = logger_.completed_source_mask();
if (discard_stage && !stager_.discard_after_success()) {
fail(TrainingCsvState::StageError, TrainingFailSite::Site18); return;
}
active_node_id_ = 0U;
transfer_window_.reset();
settle_after_source(now_ms);
}
/* A source is resolved once it is either converted into the CSV or written off.
 * The run finishes when every expected source has been resolved one way or the
 * other, so an unreachable node cannot hold the file open forever. */
void settle_after_source(uint32_t now_ms)
{
const uint8_t resolved = static_cast<uint8_t>(completed_source_mask_ | failed_source_mask_);
if (binary_only_) {
state_ = ((resolved & expected_source_mask_) == expected_source_mask_) ?
TrainingCsvState::Complete : TrainingCsvState::WaitingForNode;
return;
}
if ((resolved & expected_source_mask_) == expected_source_mask_) {
if (logger_.shutdown(now_ms)) state_ = TrainingCsvState::Complete;
else fail(TrainingCsvState::CsvError, TrainingFailSite::Site19);
} else state_ = TrainingCsvState::WaitingForNode;
}
void abandon_active_node(uint32_t now_ms)
{
const uint8_t node_id = active_node_id_;
if (node_id >= 1U && node_id <= 4U) {
failed_source_mask_ |= static_cast<uint8_t>(1U << node_id);
/* Keep the first stall visible for diagnostics without failing the run. */
if (failure_site_ == TrainingFailSite::None) {
failure_site_ = TrainingFailSite::SessionStall;
failure_node_id_ = node_id;
failure_stager_operation_ = stager_.last_operation();
failure_stager_result_ = stager_.last_result();
}
}
/* abandon_and_unlink() removes the truncated stage file but leaves discarded_
 * clear, so the node keeps its flash copy and the session can be re-pulled
 * later instead of being lost. */
(void)stager_.abandon_and_unlink();
transfer_window_.reset();
reliable_control_.reset();
active_node_id_ = 0U;
node_bno_index_ = node_icm_index_ = 0U;
++progress_seq_;
last_progress_ms_ = now_ms;
settle_after_source(now_ms);
}
void abandon_remaining_nodes(uint32_t now_ms)
{
const uint8_t unresolved = static_cast<uint8_t>(expected_source_mask_ &
~static_cast<uint8_t>(completed_source_mask_ | failed_source_mask_));
if (unresolved == 0U) return;
failed_source_mask_ |= unresolved;
if (failure_site_ == TrainingFailSite::None) {
failure_site_ = TrainingFailSite::SessionStall;
}
++progress_seq_;
last_progress_ms_ = now_ms;
settle_after_source(now_ms);
}
MasterTrainingCsvLogger logger_;
MasterNodeSessionStager stager_;
MasterNodeTransferWindow transfer_window_;
MasterNodeReliableControl reliable_control_;
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
uint8_t failed_source_mask_ = 0U;
uint8_t active_node_id_ = 0U;
uint8_t reliable_defer_services_ = 0U;
#if EXO_MASTER_BINARY_ONLY_BUILD
static constexpr bool binary_only_ = true;
#else
bool binary_only_ = false;
#endif
uint16_t file_index_ = 0U;
uint8_t cleanup_pending_mask_ = 0U;
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
}
#endif

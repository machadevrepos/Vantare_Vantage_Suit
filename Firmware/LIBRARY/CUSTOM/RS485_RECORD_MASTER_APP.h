#ifndef RS485_RECORD_MASTER_APP_H_
#define RS485_RECORD_MASTER_APP_H_

#include <stdint.h>
#include <string.h>

extern "C" {
#include "usart.h"
}

#include "BLE_RECORD_PROTOCOL.h"
#include "BLE_STREAM_V2.h"
#include "EXO_LOGGER.h"
#include "RECORDING_TYPES.h"
#include "RS485_FRAME_PROTOCOL.h"

namespace exo::rs485_record {

extern volatile uint8_t g_rs485_master_log_discovery;
extern volatile uint8_t g_rs485_master_log_transport;

#define RS485_MASTER_LOG_DISC(...) do { if (g_rs485_master_log_discovery != 0U) { EXO_LOG(__VA_ARGS__); } } while (0)
#define RS485_MASTER_LOG_XPORT(...) do { if (g_rs485_master_log_transport != 0U) { EXO_LOG(__VA_ARGS__); } } while (0)

class MasterRecordingController {
public:
    static constexpr uint32_t kDefaultDurationMs = 10000U;
    static constexpr uint64_t kDefaultLeadTimeUs = 300000ULL;

    using BleSendFn = bool (*)(const uint8_t *payload, uint8_t length);
    using TxDoneFn = void (*)();

    MasterRecordingController(UART_HandleTypeDef &uart, BleSendFn ble_send, TxDoneFn tx_done = nullptr)
        : uart_(uart), ble_send_(ble_send), tx_done_(tx_done) {}

    void begin() {
        reset_all();
        discover_nodes_startup();
    }

    void process() {
        drain_rx_ring();
        process_tx_queue();
        process_pending_ble_chunks();
        process_retries();
        probe_missing_nodes();
        process_node_download();
        emit_transfer_summary();
    }

    void on_ble_chunk_ack(uint32_t session_id, uint16_t source_id, uint32_t next_offset);

    void on_ble_reliable_ack_window(uint32_t session_id, uint16_t source_id, uint32_t next_chunk_index, uint8_t credit) {
        const int idx = index_of_node(static_cast<uint8_t>(source_id));
        if (idx < 0) {
            return;
        }
        NodeState &node = nodes_[static_cast<uint8_t>(idx)];
        if (!node.done_received || node.session_id != session_id || node.total_size == 0U) {
            EXO_LOG("[RS485][MASTER][REL] ACK_WINDOW ignored source=%u session=%lu\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id);
            return;
        }
        uint32_t offset = next_chunk_index * static_cast<uint32_t>(kChunkPayloadMax);
        if (offset > node.total_size) {
            offset = node.total_size;
        }
        if (offset < node.stream_offset) {
            node.browser_acked_offset = offset;
            node.last_browser_ack_ms = HAL_GetTick();
            node.receiver_credit = credit == 0U ? 1U : credit;
            node.next_chunk_req_ms = HAL_GetTick();
            node.download_active = true;
            state_ = State::Downloading;
            EXO_LOG("[RS485][MASTER][REL] ACK_WINDOW stale-credit source=%u session=%lu chunk=%lu off=%lu cur_off=%lu credit=%u\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id,
                    (unsigned long) next_chunk_index,
                    (unsigned long) offset,
                    (unsigned long) node.stream_offset,
                    (unsigned) node.receiver_credit);
            return;
        }
        node.stream_offset = offset;
        node.expected_offset = offset;
        if (node.recovery_request_mode && offset >= node.recovery_resume_offset) {
            node.recovery_request_mode = false;
            node.recovery_chunks_remaining = 0U;
            node.last_req_wildcard_seq = true;
            EXO_LOG("[RS485][MASTER][REL] recovery exit source=%u session=%lu off=%lu resume=%lu\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id,
                    (unsigned long) offset,
                    (unsigned long) node.recovery_resume_offset);
        }
        if (node.max_seen_offset < offset) {
            node.max_seen_offset = offset;
        }
        node.browser_acked_offset = offset;
        node.expected_chunk_seq = static_cast<uint16_t>(next_chunk_index + 1U);
        node.receiver_credit = credit == 0U ? 1U : credit;
        node.download_active = true;
        node.transfer_aborted = false;
        if (!node.recovery_request_mode) {
            node.last_req_wildcard_seq = true;
        }
        node.same_offset_retry_count = 0U;
        node.next_chunk_req_ms = HAL_GetTick();
        node.last_progress_ms = HAL_GetTick();
        clear_cached_chunk(node);
        pending_[static_cast<uint8_t>(idx)].active = false;
        state_ = State::Downloading;
        transfer_hold_ = false;
        EXO_LOG("[RS485][MASTER][REL] ACK_WINDOW source=%u session=%lu chunk=%lu off=%lu credit=%u\r\n",
                (unsigned) source_id,
                (unsigned long) session_id,
                (unsigned long) next_chunk_index,
                (unsigned long) offset,
                (unsigned) node.receiver_credit);
    }

    void on_ble_reliable_nack_range(uint32_t session_id, uint16_t source_id, uint32_t first_chunk_index, uint8_t count) {
        const int idx = index_of_node(static_cast<uint8_t>(source_id));
        if (idx < 0) {
            return;
        }
        NodeState &node = nodes_[static_cast<uint8_t>(idx)];
        if (!node.done_received || node.session_id != session_id || node.total_size == 0U) {
            EXO_LOG("[RS485][MASTER][REL] NACK_RANGE ignored source=%u session=%lu\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id);
            return;
        }
        uint32_t offset = first_chunk_index * static_cast<uint32_t>(kChunkPayloadMax);
        if (offset > node.total_size) {
            offset = node.total_size;
        }
        const bool cached_match = node.ble_pending_active &&
                                  node.ble_pending_source_id == source_id &&
                                  node.ble_pending_offset == offset;
        node.recovery_resume_offset = node.stream_offset;
        node.expected_offset = offset;
        node.expected_chunk_seq = static_cast<uint16_t>(first_chunk_index + 1U);
        node.recovery_chunks_remaining = count == 0U ? 1U : count;
        node.receiver_credit = node.recovery_chunks_remaining;
        node.download_active = true;
        node.transfer_aborted = false;
        node.recovery_request_mode = true;
        node.last_req_wildcard_seq = true;
        node.same_offset_retry_count = 0U;
        node.next_chunk_req_ms = HAL_GetTick();
        node.last_progress_ms = HAL_GetTick();
        if (cached_match) {
            node.ble_next_retry_ms = HAL_GetTick();
        } else {
            clear_cached_chunk(node);
        }
        pending_[static_cast<uint8_t>(idx)].active = false;
        state_ = State::Downloading;
        transfer_hold_ = false;
        EXO_LOG("[RS485][MASTER][REL] NACK_RANGE source=%u session=%lu first=%lu off=%lu count=%u resume=%lu cached=%u\r\n",
                (unsigned) source_id,
                (unsigned long) session_id,
                (unsigned long) first_chunk_index,
                (unsigned long) offset,
                (unsigned) node.recovery_chunks_remaining,
                (unsigned long) node.recovery_resume_offset,
                (unsigned) cached_match);
    }

    void on_ble_reliable_pause(uint32_t session_id, uint16_t source_id) {
        const int idx = index_of_node(static_cast<uint8_t>(source_id));
        if (idx < 0) {
            return;
        }
        NodeState &node = nodes_[static_cast<uint8_t>(idx)];
        if (node.session_id == session_id) {
            node.receiver_credit = 0U;
            node.download_active = false;
            pending_[static_cast<uint8_t>(idx)].active = false;
            clear_cached_chunk(node);
            EXO_LOG("[RS485][MASTER][REL] PAUSE source=%u session=%lu\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id);
        }
    }

    void on_ble_reliable_verify_ok(uint32_t session_id, uint16_t source_id, uint32_t file_crc32) {
        const int idx = index_of_node(static_cast<uint8_t>(source_id));
        if (idx < 0) {
            return;
        }
        NodeState &node = nodes_[static_cast<uint8_t>(idx)];
        if (node.session_id != session_id || node.payload_crc32 != file_crc32) {
            EXO_LOG("[RS485][MASTER][REL] VERIFY_OK ignored source=%u session=%lu crc=0x%08lX expect=0x%08lX\r\n",
                    (unsigned) source_id,
                    (unsigned long) session_id,
                    (unsigned long) file_crc32,
                    (unsigned long) node.payload_crc32);
            return;
        }
        exo::SessionCompleteAckMessage ack{};
        ack.command = exo::RecordCommand::SessionCompleteAck;
        ack.session_id = node.session_id;
        ack.payload_crc32 = node.payload_crc32;
        (void)send_frame(static_cast<uint8_t>(source_id), MsgType::TransferCompleteAck,
                         reinterpret_cast<const uint8_t *>(&ack), sizeof(ack), false, false, 0U);
        node.receiver_credit = 0U;
        node.download_active = false;
        node.browser_complete = true;
        if (!any_download_active()) {
            state_ = State::Idle;
            start_request_active_ = false;
            last_closed_session_id_ = last_start_.session_id;
            have_last_closed_session_ = true;
        }
        EXO_LOG("[RS485][MASTER][REL] COMMIT_DONE source=%u session=%lu crc=0x%08lX\r\n",
                (unsigned) source_id,
                (unsigned long) session_id,
                (unsigned long) file_crc32);
    }

    void on_uart_rx_chunk(const uint8_t *data, uint16_t len) {
        if (data == nullptr || len == 0U) {
            return;
        }
        RS485_MASTER_LOG_XPORT("[RS485][MASTER][RXDMA] size=%u\r\n", (unsigned) len);
        for (uint16_t i = 0U; i < len; ++i) {
            const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxRingSize);
            if (next == rx_tail_) {
                rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxRingSize);
            }
            rx_ring_[rx_head_] = data[i];
            rx_head_ = next;
        }
        // Parse/process in main loop context (process()), not in UART RX callback context.
    }

    bool pop_record_done(exo::RecordDoneMessage &out) {
        if (!have_record_done_) {
            return false;
        }
        out = last_record_done_;
        have_record_done_ = false;
        return true;
    }

    bool pop_next_record_done(exo::RecordDoneMessage &out) {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            NodeState &node = nodes_[i];
            if (!node.done_received ||
                node.browser_complete ||
                node.manifest_announced ||
                node.session_id == 0U ||
                node.total_size == 0U) {
                continue;
            }
            memset(&out, 0, sizeof(out));
            out.command = exo::RecordCommand::RecordDone;
            out.session_id = node.session_id;
            out.node_id = kNodeIds[i];
            out.total_size = node.total_size;
            out.payload_crc32 = node.payload_crc32;
            out.actual_duration_ms = node.actual_duration_ms;
            node.manifest_announced = true;
            if (have_record_done_ && last_record_done_.node_id == out.node_id && last_record_done_.session_id == out.session_id) {
                have_record_done_ = false;
            }
            return true;
        }
        return false;
    }

    bool start_or_record_active() const {
        return start_request_active_ || (state_ != State::Idle);
    }

    void set_transfer_hold(bool hold) {
        transfer_hold_ = hold;
    }

    bool start_from_ble(const exo::StartRecordMessage &msg) {
        if (msg.command != exo::RecordCommand::StartRecord) {
            return false;
        }
        if (start_request_active_) {
            EXO_LOG("[RS485][MASTER][START] reject: start already active session=%lu\r\n",
                    (unsigned long) last_start_.session_id);
            return false;
        }
        exo::StartRecordMessage effective_msg = msg;
        if (have_last_closed_session_ && effective_msg.session_id <= last_closed_session_id_) {
            const uint32_t requested = effective_msg.session_id;
            effective_msg.session_id = last_closed_session_id_ + 1U;
            EXO_LOG("[RS485][MASTER][START] session remap %lu -> %lu\r\n",
                    (unsigned long) requested,
                    (unsigned long) effective_msg.session_id);
        }
        if (effective_msg.requested_duration_ms == 0U) {
            effective_msg.requested_duration_ms = kDefaultDurationMs;
        }
        if (effective_msg.start_timestamp_us == 0ULL) {
            effective_msg.start_timestamp_us = kDefaultLeadTimeUs;
        }
        start_request_active_ = true;
        last_start_ = effective_msg;
        active_node_count_ = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i].ready = false;
            nodes_[i].burst_supported = false;
            nodes_[i].await_start_ack = false;
            nodes_[i].done_received = false;
            nodes_[i].download_active = false;
            nodes_[i].expected_offset = 0U;
            nodes_[i].stream_offset = 0U;
            nodes_[i].recovery_resume_offset = 0U;
            nodes_[i].max_seen_offset = 0U;
            nodes_[i].session_id = effective_msg.session_id;
            nodes_[i].actual_duration_ms = 0U;
            nodes_[i].payload_crc32 = 0U;
            nodes_[i].expected_chunk_seq = 1U;
            nodes_[i].recovery_request_mode = false;
            nodes_[i].last_req_wildcard_seq = false;
            nodes_[i].seq_resync_count = 0U;
            nodes_[i].last_req_offset = 0U;
            nodes_[i].last_req_size = 0U;
            nodes_[i].same_offset_retry_count = 0U;
            nodes_[i].recovery_window_start_ms = 0U;
            nodes_[i].recovery_cycles_in_window = 0U;
            nodes_[i].recovery_chunks_remaining = 0U;
            nodes_[i].receiver_credit = 0U;
            nodes_[i].last_progress_ms = 0U;
            nodes_[i].recovery_events = 0U;
            nodes_[i].crc_fail_count = 0U;
            nodes_[i].seq_mismatch_count = 0U;
            nodes_[i].offset_mismatch_count = 0U;
            nodes_[i].wire_len_fail_count = 0U;
            nodes_[i].duplicate_count = 0U;
            nodes_[i].nack_count = 0U;
            nodes_[i].consecutive_bad_chunks = 0U;
            nodes_[i].transfer_aborted = false;
            nodes_[i].bno_remaining = 0U;
            nodes_[i].icm_remaining = 0U;
            nodes_[i].browser_acked_offset = 0U;
            nodes_[i].last_browser_ack_ms = 0U;
            nodes_[i].browser_complete = false;
            nodes_[i].manifest_announced = false;
            nodes_[i].stream_sensor = 0U;
            nodes_[i].partial_len = 0U;
            memset(&nodes_[i].session_hdr, 0, sizeof(nodes_[i].session_hdr));
        }
        if (!query_nodes_ready()) {
            EXO_LOG("[RS485][MASTER][START] abort: one or more nodes not ready\r\n");
            abort_start_request();
            return false;
        }
        state_ = State::WaitStartAck;
        if (!send_start_to_all()) {
            abort_start_request();
            return false;
        }
        return true;
    }

    uint8_t discovered_count() const {
        uint8_t count = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].discovered) {
                ++count;
            }
        }
        return count;
    }

    uint8_t copy_discovered_node_ids(uint8_t *out_ids, uint8_t out_cap) const {
        if (out_ids == nullptr || out_cap == 0U) {
            return 0U;
        }
        uint8_t written = 0U;
        for (uint8_t i = 0U; i < kNodeCount && written < out_cap; ++i) {
            if (nodes_[i].discovered) {
                out_ids[written++] = kNodeIds[i];
            }
        }
        return written;
    }

    bool request_node_id(uint8_t node_id) {
        if (index_of_node(node_id) < 0) {
            return false;
        }
        return send_with_ack(node_id, MsgType::GetNodeIdReq, nullptr, 0U);
    }

    bool provision_node_id(uint8_t current_id, uint8_t new_id) {
        if (index_of_node(current_id) < 0 || index_of_node(new_id) < 0) {
            return false;
        }
        SetNodeIdWire req{};
        req.new_id = new_id;
        return send_with_ack(current_id, MsgType::SetNodeIdReq, reinterpret_cast<const uint8_t *>(&req), sizeof(req));
    }

    void rediscover_nodes() {
        discover_nodes_startup();
    }

    bool reset_and_abort_all(bool erase_remote) {
        bool ok = true;
        if (erase_remote) {
            ok = send_reset_to_active_nodes_and_wait_ack();
        }
        clear_runtime_state_only();
        return ok;
    }

private:
    static const char* msg_type_str(MsgType msg) {
        switch (msg) {
            case MsgType::Ack: return "ACK";
            case MsgType::Nack: return "NACK";
            case MsgType::DiscoverReq: return "DISC_REQ";
            case MsgType::DiscoverResp: return "DISC_RSP";
            case MsgType::SetNodeIdReq: return "SET_NODE_ID";
            case MsgType::GetNodeIdReq: return "GET_NODE_ID";
            case MsgType::NodeIdResp: return "NODE_ID_RSP";
            case MsgType::StartRecord: return "START_REC";
            case MsgType::RecordStatus: return "REC_STATUS";
            case MsgType::DataChunkReq: return "CHUNK_REQ";
            case MsgType::DataChunk: return "CHUNK";
            case MsgType::TransferCompleteAck: return "XFER_DONE_ACK";
            case MsgType::ResetState: return "RESET_STATE";
            case MsgType::GetManifest: return "GET_MANIFEST";
            case MsgType::Manifest: return "MANIFEST";
            case MsgType::RequestChunk: return "REQ_CHUNK";
            case MsgType::DataRangeReq: return "RANGE_REQ";
            case MsgType::DataAckBitmap: return "ACK_BITMAP";
            case MsgType::PauseTransfer: return "PAUSE";
            case MsgType::ResumeTransfer: return "RESUME";
            case MsgType::CancelTransfer: return "CANCEL";
            case MsgType::CommitDone: return "COMMIT_DONE";
            case MsgType::Error: return "ERROR";
            default: return "UNK";
        }
    }

    static bool time_reached(uint32_t now, uint32_t due_ms) {
        return static_cast<int32_t>(now - due_ms) >= 0;
    }

    static constexpr uint8_t kMasterNodeId = 0x00U;
    static constexpr uint8_t kNodeIds[5] = {1U, 2U, 3U, 4U, 5U};
    static constexpr uint8_t kNodeCount = static_cast<uint8_t>(sizeof(kNodeIds) / sizeof(kNodeIds[0]));
    static constexpr uint32_t kAckTimeoutMs = 900U;
    static constexpr uint32_t kStartRecordAckTimeoutMs = 8000U;
    static constexpr uint8_t kMaxRetries = 10U;
    // Keep reliable BLE frames under ATT payload limits: 25-byte header + 180-byte payload.
    static constexpr uint16_t kChunkPayloadMax = 180U;
    static constexpr uint32_t kDiscoverTimeoutMs = 1000U;
    static constexpr uint32_t kPostDoneDownloadDelayMs = 250U;
    /* THVD1426 auto-direction half-duplex: keep larger turn-around/pacing margins. */
    static constexpr uint32_t kRs485Baud = 921600U;
    static constexpr uint32_t kCharTimeUs = 11U;
    static constexpr uint32_t kTxToRxGuardUs = 25U;
    static constexpr uint32_t kRxToTxGuardUs = 25U;
    static constexpr uint32_t kInterTransactionGapUs = 50U;
    static constexpr uint32_t kInterChunkDelayMs = 1U;
    static constexpr uint32_t kInterChunkDelayMinMs = 0U;
    static constexpr uint32_t kInterChunkDelayMaxMs = 120U;
    static constexpr uint32_t kBusyRetryDelayMs = 4U;
    static constexpr uint32_t kHalfDuplexInterFrameMs = 1U;
    static constexpr uint32_t kSummaryPeriodMs = 1000U;
    static constexpr uint32_t kRediscoverIntervalMs = 2000U;
    static constexpr uint16_t kRxRingSize = 1024U;
    static constexpr uint32_t kDownloadStallTimeoutMs = 5000U;
    static constexpr uint16_t kRecoveryWindowBytes = static_cast<uint16_t>(kChunkPayloadMax * 8U);
    static constexpr uint8_t kMaxConsecutiveBadChunks = 20U;
    static constexpr uint8_t kMaxSameOffsetRetries = 16U;
    static constexpr uint8_t kMaxRecoveryCyclesPerWindow = 24U;
    static constexpr uint32_t kRecoveryWindowMs = 3000U;
    static constexpr uint8_t kTxQueueDepth = 16U;

    enum class State : uint8_t {
        Idle,
        WaitStartAck,
        WaitNodeDone,
        Downloading
    };

    struct PendingTx {
        bool active = false;
        uint8_t node_id = 0U;
        uint8_t seq = 0U;
        MsgType msg = MsgType::Error;
        uint8_t frame_buf[kMaxFrameBytes] = {0U};
        uint16_t frame_len = 0U;
        uint32_t last_tx_ms = 0U;
        uint32_t ack_timeout_ms = kAckTimeoutMs;
        uint8_t retries = 0U;
        bool tx_started = false;
    };

    struct TxQueueEntry {
        bool active = false;
        uint8_t raw[kMaxFrameBytes] = {0U};
        uint16_t raw_len = 0U;
        uint8_t priority = 2U;
        uint8_t dst_id = 0xFFU;
        uint8_t seq = 0U;
        MsgType msg = MsgType::Error;
    };

    struct NodeState {
        bool discovered = false;
        bool ready = false;
        bool burst_supported = false;
        bool await_start_ack = false;
        bool done_received = false;
        bool download_active = false;
        uint32_t session_id = 0U;
        uint32_t actual_duration_ms = 0U;
        uint32_t expected_offset = 0U;
        uint32_t stream_offset = 0U;
        uint32_t recovery_resume_offset = 0U;
        uint32_t max_seen_offset = 0U;
        uint32_t total_size = 0U;
        uint32_t payload_crc32 = 0U;
        uint16_t expected_chunk_seq = 1U;
        bool recovery_request_mode = false;
        bool last_req_wildcard_seq = false;
        uint32_t seq_resync_count = 0U;
        uint32_t last_progress_ms = 0U;
        uint32_t recovery_events = 0U;
        uint32_t crc_fail_count = 0U;
        uint32_t seq_mismatch_count = 0U;
        uint32_t offset_mismatch_count = 0U;
        uint32_t wire_len_fail_count = 0U;
        uint32_t duplicate_count = 0U;
        uint32_t nack_count = 0U;
        uint8_t consecutive_bad_chunks = 0U;
        bool transfer_aborted = false;
        exo::SessionHeader session_hdr{};
        uint32_t bno_remaining = 0U;
        uint32_t icm_remaining = 0U;
        uint32_t next_chunk_req_ms = 0U;
        uint32_t browser_acked_offset = 0U;
        uint32_t last_browser_ack_ms = 0U;
        bool browser_complete = false;
        bool manifest_announced = false;
        uint8_t stream_sensor = 0U;
        uint8_t partial[128] = {0U};
        uint16_t partial_len = 0U;
        uint8_t ble_pending_payload[kChunkPayloadMax] = {0U};
        uint16_t ble_pending_len = 0U;
        uint16_t ble_pending_source_id = 0U;
        uint32_t ble_pending_offset = 0U;
        uint8_t ble_pending_sequence = 0U;
        bool ble_pending_active = false;
        uint32_t ble_next_retry_ms = 0U;
        uint32_t ble_busy_count = 0U;
        uint32_t chunk_rerequest_count = 0U;
        uint32_t chunk_accept_count = 0U;
        uint32_t session_bytes_sent = 0U;
        uint32_t session_start_ms = 0U;
        uint32_t adaptive_delay_ms = kInterChunkDelayMs;
        uint32_t last_req_offset = 0U;
        uint16_t last_req_size = 0U;
        uint8_t same_offset_retry_count = 0U;
        uint32_t recovery_window_start_ms = 0U;
        uint8_t recovery_cycles_in_window = 0U;
        uint8_t recovery_chunks_remaining = 0U;
        uint8_t receiver_credit = 0U;
    };

    void adjust_adaptive_delay(NodeState &node, bool busy) {
        if (busy) {
            if (node.adaptive_delay_ms + 4U > kInterChunkDelayMaxMs) {
                node.adaptive_delay_ms = kInterChunkDelayMaxMs;
            } else {
                node.adaptive_delay_ms = static_cast<uint32_t>(node.adaptive_delay_ms + 4U);
            }
            return;
        }
        if (node.adaptive_delay_ms > kInterChunkDelayMinMs) {
            node.adaptive_delay_ms--;
        }
    }

    bool node_has_pending_chunk(const NodeState &node, uint8_t source_id) const {
        const int idx = index_of_node(source_id);
        if (idx < 0) {
            return false;
        }
        const PendingTx &p = pending_[static_cast<uint8_t>(idx)];
        return p.active &&
               p.msg == MsgType::DataChunkReq &&
               p.seq == node.ble_pending_sequence;
    }

    void cache_pending_chunk(NodeState &node, uint16_t source_id, uint32_t offset, uint8_t sequence,
                             const uint8_t *payload, uint16_t payload_len) {
        if (payload_len > kChunkPayloadMax || payload == nullptr) {
            return;
        }
        memcpy(node.ble_pending_payload, payload, payload_len);
        node.ble_pending_len = payload_len;
        node.ble_pending_source_id = source_id;
        node.ble_pending_offset = offset;
        node.ble_pending_sequence = sequence;
        node.ble_pending_active = true;
        node.ble_next_retry_ms = HAL_GetTick() + kBusyRetryDelayMs;
    }

    void clear_cached_chunk(NodeState &node) {
        node.ble_pending_active = false;
        node.ble_pending_len = 0U;
        node.ble_pending_source_id = 0U;
        node.ble_pending_offset = 0U;
        node.ble_pending_sequence = 0U;
    }

    void process_pending_ble_chunks() {
        const uint32_t now = HAL_GetTick();
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            NodeState &node = nodes_[i];
            if (!node.ble_pending_active || !node.download_active) {
                continue;
            }
            if (!time_reached(now, node.ble_next_retry_ms)) {
                continue;
            }
            if (!send_ble_session_chunk(node,
                                        node.ble_pending_source_id,
                                        node.ble_pending_offset,
                                        node.ble_pending_sequence,
                                        node.ble_pending_payload,
                                        node.ble_pending_len)) {
                node.ble_busy_count++;
                node.chunk_rerequest_count++;
                adjust_adaptive_delay(node, true);
                node.ble_next_retry_ms = now + kBusyRetryDelayMs;
                continue;
            }
            adjust_adaptive_delay(node, false);
            EXO_LOG("[RS485][MASTER][BLE] retry sent node=%u off=%lu size=%u cache_pending=0\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned long) node.ble_pending_offset,
                    (unsigned) node.ble_pending_len);
            clear_cached_chunk(node);
            if (node.recovery_request_mode && node.recovery_chunks_remaining > 0U) {
                node.recovery_chunks_remaining = 0U;
                if (node.expected_offset < node.recovery_resume_offset) {
                    node.expected_offset = node.recovery_resume_offset;
                }
                node.expected_chunk_seq = static_cast<uint16_t>((node.expected_offset / static_cast<uint32_t>(kChunkPayloadMax)) + 1U);
                node.recovery_request_mode = false;
                node.last_req_wildcard_seq = false;
                EXO_LOG("[RS485][MASTER][RECOVER] exit after pending_ble node=%u session=%lu off=%lu\r\n",
                        (unsigned) kNodeIds[i],
                        (unsigned long) node.session_id,
                        (unsigned long) node.expected_offset);
            }
        }
    }

    void emit_transfer_summary() {
        const uint32_t now = HAL_GetTick();
        if (!time_reached(now, next_summary_ms_)) {
            return;
        }
        next_summary_ms_ = now + kSummaryPeriodMs;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            NodeState &node = nodes_[i];
            if (!node.download_active || node.session_id == 0U) {
                continue;
            }
            const uint32_t elapsed_ms = (node.session_start_ms == 0U) ? 0U : (now - node.session_start_ms);
            const uint32_t bytes_per_sec = (elapsed_ms > 0U)
                                               ? static_cast<uint32_t>((static_cast<uint64_t>(node.session_bytes_sent) * 1000ULL) / elapsed_ms)
                                               : 0U;
            EXO_LOG("[RS485][MASTER][SUMMARY] node=%u session=%lu sent=%lu/%lu busy=%lu accept=%lu retry=%lu rate=%luBps delay=%lums\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned long) node.session_id,
                    (unsigned long) node.session_bytes_sent,
                    (unsigned long) node.total_size,
                    (unsigned long) node.ble_busy_count,
                    (unsigned long) node.chunk_accept_count,
                    (unsigned long) node.chunk_rerequest_count,
                    (unsigned long) bytes_per_sec,
                    (unsigned long) node.adaptive_delay_ms);
            EXO_LOG("[RS485][MASTER][SUMMARY2] node=%u recover=%lu dup=%lu crc_fail=%lu seq_fail=%lu off_fail=%lu len_fail=%lu nack=%lu bad_streak=%u\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned long) node.recovery_events,
                    (unsigned long) node.duplicate_count,
                    (unsigned long) node.crc_fail_count,
                    (unsigned long) node.seq_mismatch_count,
                    (unsigned long) node.offset_mismatch_count,
                    (unsigned long) node.wire_len_fail_count,
                    (unsigned long) node.nack_count,
                    (unsigned) node.consecutive_bad_chunks);
            EXO_LOG("[RS485][MASTER][SUMMARY3] node=%u seq_resync=%lu recover_mode=%u wildcard=%u\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned long) node.seq_resync_count,
                    (unsigned) (node.recovery_request_mode ? 1U : 0U),
                    (unsigned) (node.last_req_wildcard_seq ? 1U : 0U));
            EXO_LOG("[RS485][MASTER][SUMMARY4] node=%u off_expected=%lu off_req=%lu req_sz=%u stuck_retry=%u rec_win=%u/%u\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned long) node.expected_offset,
                    (unsigned long) node.last_req_offset,
                    (unsigned) node.last_req_size,
                    (unsigned) node.same_offset_retry_count,
                    (unsigned) node.recovery_cycles_in_window,
                    (unsigned) kMaxRecoveryCyclesPerWindow);
        }
    }

    bool send_start_to_all() {
        exo::StartRecordMessage msg = last_start_;
        EXO_LOG("[RS485][MASTER][START] fanout nodes=%u session=%lu\r\n",
                (unsigned) active_node_count_, (unsigned long) msg.session_id);
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (!nodes_[i].discovered) {
                continue;
            }
            nodes_[i].await_start_ack = true;
            if (!send_with_ack(kNodeIds[i], MsgType::StartRecord, reinterpret_cast<const uint8_t *>(&msg), sizeof(msg))) {
                EXO_LOG("[RS485][MASTER][START] send fail node=%u\r\n", (unsigned) kNodeIds[i]);
                return false;
            }
        }
        return true;
    }

    void discover_nodes_startup() {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i].discovered = false;
            nodes_[i].ready = false;
            nodes_[i].burst_supported = false;
            RS485_MASTER_LOG_DISC("[RS485][MASTER][DISC] send req node=%u\r\n", (unsigned) kNodeIds[i]);
            (void)send_with_ack(kNodeIds[i], MsgType::DiscoverReq, nullptr, 0U);
        }
        const uint32_t start_ms = HAL_GetTick();
        while ((HAL_GetTick() - start_ms) < kDiscoverTimeoutMs) {
            drain_rx_ring();
            process_retries();
            bool all_discovered = true;
            bool any_pending = false;
            for (uint8_t i = 0U; i < kNodeCount; ++i) {
                all_discovered &= nodes_[i].discovered;
                any_pending |= pending_[i].active;
            }
            if (all_discovered && !any_pending) {
                break;
            }
        }
    }

    bool query_nodes_ready() {
        uint8_t discovered_targets = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i].ready = false;
            nodes_[i].burst_supported = false;
            nodes_[i].await_start_ack = false;
            pending_[i].active = false;
            if (!nodes_[i].discovered) {
                continue;
            }
            discovered_targets++;
            EXO_LOG("[RS485][MASTER][READY] query node=%u\r\n", (unsigned) kNodeIds[i]);
            (void)send_with_ack(kNodeIds[i], MsgType::DiscoverReq, nullptr, 0U);
        }
        if (discovered_targets == 0U) {
            active_node_count_ = 0U;
            EXO_LOG("[RS485][MASTER][READY] no discovered nodes eligible for start\r\n");
            return false;
        }

        const uint32_t start_ms = HAL_GetTick();
        while ((HAL_GetTick() - start_ms) < kDiscoverTimeoutMs) {
            drain_rx_ring();
            process_retries();
            uint8_t ready_count = 0U;
            for (uint8_t i = 0U; i < kNodeCount; ++i) {
                if (nodes_[i].discovered && nodes_[i].ready) {
                    ++ready_count;
                }
            }
            if (ready_count > 0U) {
                for (uint8_t j = 0U; j < kNodeCount; ++j) {
                    if (!(nodes_[j].discovered && nodes_[j].ready)) {
                        pending_[j].active = false;
                        nodes_[j].await_start_ack = false;
                    }
                }
                active_node_count_ = ready_count;
                return true;
            }
        }

        active_node_count_ = 0U;

        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            EXO_LOG("[RS485][MASTER][READY] node=%u ready=%u discovered=%u\r\n",
                    (unsigned) kNodeIds[i],
                    (unsigned) nodes_[i].ready,
                    (unsigned) nodes_[i].discovered);
        }
        /* Fallback: if at least one node is in discovered set, proceed without strict ready bit gate. */
        uint8_t discovered_count = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].discovered) {
                nodes_[i].ready = true;
                discovered_count++;
            }
        }
        if (discovered_count > 0U) {
            active_node_count_ = discovered_count;
            EXO_LOG("[RS485][MASTER][READY] fallback proceed with discovered nodes=%u\r\n",
                    (unsigned) discovered_count);
            return true;
        }
        return false;
    }


    void process_retries() {
        process_tx_queue();
        const uint32_t now = HAL_GetTick();
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (!pending_[i].active) {
                continue;
            }
            if ((now - pending_[i].last_tx_ms) < pending_[i].ack_timeout_ms) {
                continue;
            }
            if (pending_[i].retries >= kMaxRetries) {
                RS485_MASTER_LOG_DISC("[RS485][MASTER][RETRY] giveup node=%u seq=%u msg=%s\r\n",
                        (unsigned) pending_[i].node_id,
                        (unsigned) pending_[i].seq,
                        msg_type_str(pending_[i].msg));
                if (pending_[i].msg == MsgType::StartRecord) {
                    nodes_[i].await_start_ack = false;
                    if (state_ == State::WaitStartAck && !all_start_acked()) {
                        EXO_LOG("[RS485][MASTER][START] node=%u start ACK timeout\r\n",
                                (unsigned) pending_[i].node_id);
                    }
                }
                if (pending_[i].msg == MsgType::DataChunkReq || pending_[i].msg == MsgType::DataRangeReq) {
                    nodes_[i].download_active = false;
                    nodes_[i].receiver_credit = 0U;
                    nodes_[i].recovery_request_mode = true;
                    EXO_LOG("[RS485][MASTER][REL] pause node=%u session=%lu reason=chunk_request_timeout\r\n",
                            (unsigned) pending_[i].node_id,
                            (unsigned long) nodes_[i].session_id);
                }
                pending_[i].active = false;
                if (state_ == State::WaitStartAck && all_start_acked()) {
                    state_ = State::WaitNodeDone;
                }
                if (state_ == State::WaitNodeDone) {
                    bool any_done = false;
                    for (uint8_t n = 0U; n < kNodeCount; ++n) {
                        if (nodes_[n].done_received) {
                            any_done = true;
                            break;
                        }
                    }
                    if (!any_done && !any_pending_active()) {
                        start_request_active_ = false;
                        state_ = State::Idle;
                        last_closed_session_id_ = last_start_.session_id;
                        have_last_closed_session_ = true;
                    }
                }
                if (state_ == State::Downloading && !any_download_active() && !any_pending_active()) {
                    start_request_active_ = false;
                    state_ = State::Idle;
                    last_closed_session_id_ = last_start_.session_id;
                    have_last_closed_session_ = true;
                }
                continue;
            }
            if (!pending_[i].tx_started) {
                continue;
            }
            if (!queue_raw_tx(pending_[i].frame_buf, pending_[i].frame_len, tx_priority_for(pending_[i].msg), pending_[i].node_id, pending_[i].seq, pending_[i].msg)) {
                continue;
            }
            RS485_MASTER_LOG_DISC("[RS485][MASTER][RETRY] node=%u seq=%u msg=%s try=%u\r\n",
                    (unsigned) pending_[i].node_id,
                    (unsigned) pending_[i].seq,
                    msg_type_str(pending_[i].msg),
                    (unsigned) (pending_[i].retries + 1U));
            pending_[i].last_tx_ms = now;
            pending_[i].retries++;
            pending_[i].tx_started = false;
        }
    }

    void process_tx_queue() {
        const uint32_t now = HAL_GetTick();
        if (!time_reached(now, next_tx_allowed_ms_)) {
            return;
        }
        if (tx_queue_head_ == tx_queue_tail_) {
            return;
        }
        const int selected = select_tx_queue_index();
        if (selected < 0) {
            return;
        }
        TxQueueEntry &entry = tx_queue_[static_cast<uint8_t>(selected)];
        if (!entry.active || entry.raw_len == 0U) {
            entry = TxQueueEntry{};
            advance_tx_tail();
            return;
        }
        rs485_guard_us(kRxToTxGuardUs);
        const HAL_StatusTypeDef tx = HAL_UART_Transmit(&uart_, entry.raw, entry.raw_len, 20U);
        if (tx_done_ != nullptr) {
            tx_done_();
        }
        if (tx == HAL_OK) {
            mark_pending_tx_started(entry);
            RS485_MASTER_LOG_XPORT("[RS485][MASTER][TXN] tx node=%u msg=%s seq=%u prio=%u q=%u\r\n",
                    (unsigned) entry.dst_id,
                    msg_type_str(entry.msg),
                    (unsigned) entry.seq,
                    (unsigned) entry.priority,
                    (unsigned) tx_queue_count());
            rs485_guard_us(kTxToRxGuardUs);
            entry = TxQueueEntry{};
            advance_tx_tail();
            next_tx_allowed_ms_ = HAL_GetTick() + kHalfDuplexInterFrameMs;
        }
    }

    bool queue_raw_tx(const uint8_t *raw, uint16_t raw_len, uint8_t priority = 2U,
                      uint8_t dst_id = 0xFFU, uint8_t seq = 0U, MsgType msg = MsgType::Error) {
        const uint8_t next = static_cast<uint8_t>((tx_queue_head_ + 1U) % kTxQueueDepth);
        if (next == tx_queue_tail_) {
            RS485_MASTER_LOG_XPORT("[RS485][MASTER][TXQ] overflow len=%u\r\n", (unsigned) raw_len);
            return false;
        }
        TxQueueEntry &entry = tx_queue_[tx_queue_head_];
        entry = TxQueueEntry{};
        entry.active = true;
        entry.raw_len = raw_len;
        entry.priority = priority;
        entry.dst_id = dst_id;
        entry.seq = seq;
        entry.msg = msg;
        memcpy(entry.raw, raw, raw_len);
        tx_queue_head_ = next;
        return true;
    }

    uint8_t tx_priority_for(MsgType msg) const {
        if (msg == MsgType::DataChunkReq || msg == MsgType::DataRangeReq) {
            return 0U;
        }
        if (msg == MsgType::Ack || msg == MsgType::Nack) {
            return 1U;
        }
        return 2U;
    }

    int select_tx_queue_index() const {
        int selected = -1;
        uint8_t best_prio = 0xFFU;
        uint8_t idx = tx_queue_tail_;
        while (idx != tx_queue_head_) {
            const TxQueueEntry &entry = tx_queue_[idx];
            if (entry.active && entry.priority < best_prio) {
                best_prio = entry.priority;
                selected = static_cast<int>(idx);
                if (best_prio == 0U) {
                    break;
                }
            }
            idx = static_cast<uint8_t>((idx + 1U) % kTxQueueDepth);
        }
        return selected;
    }

    uint8_t tx_queue_count() const {
        uint8_t count = 0U;
        uint8_t idx = tx_queue_tail_;
        while (idx != tx_queue_head_) {
            if (tx_queue_[idx].active) {
                count++;
            }
            idx = static_cast<uint8_t>((idx + 1U) % kTxQueueDepth);
        }
        return count;
    }

    void advance_tx_tail() {
        while (tx_queue_tail_ != tx_queue_head_ && !tx_queue_[tx_queue_tail_].active) {
            tx_queue_tail_ = static_cast<uint8_t>((tx_queue_tail_ + 1U) % kTxQueueDepth);
        }
    }

    void mark_pending_tx_started(const TxQueueEntry &entry) {
        const int idx = index_of_node(entry.dst_id);
        if (idx < 0) {
            return;
        }
        PendingTx &p = pending_[static_cast<uint8_t>(idx)];
        if (p.active && p.seq == entry.seq && p.msg == entry.msg) {
            p.last_tx_ms = HAL_GetTick();
            p.tx_started = true;
        }
    }

    static void rs485_guard_us(uint32_t us) {
        const uint32_t loops = ((SystemCoreClock / 1000000U) * us) / 6U;
        for (volatile uint32_t i = 0U; i < loops; ++i) {
            __NOP();
        }
    }

    void probe_missing_nodes() {
        if (state_ != State::Idle || start_request_active_ || any_pending_active()) {
            return;
        }
        const uint32_t now = HAL_GetTick();
        if ((now - last_rediscover_ms_) < kRediscoverIntervalMs) {
            return;
        }
        last_rediscover_ms_ = now;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (!nodes_[i].discovered) {
                RS485_MASTER_LOG_DISC("[RS485][MASTER][DISC] reprobe node=%u\r\n", (unsigned) kNodeIds[i]);
                (void)send_with_ack(kNodeIds[i], MsgType::DiscoverReq, nullptr, 0U);
            }
        }
    }

    void process_node_download() {
        if (transfer_hold_) {
            return;
        }
        const uint32_t now = HAL_GetTick();
        if (state_ == State::WaitStartAck) {
            if (all_start_acked()) {
                EXO_LOG("[RS485][MASTER][STATE] WaitStartAck -> WaitNodeDone\r\n");
                state_ = State::WaitNodeDone;
            }
        }

        if (state_ == State::WaitNodeDone) {
            for (uint8_t i = 0U; i < kNodeCount; ++i) {
                if (nodes_[i].done_received && !nodes_[i].download_active && nodes_[i].receiver_credit > 0U) {
                    if (!time_reached(now, nodes_[i].next_chunk_req_ms)) {
                        continue;
                    }
                    nodes_[i].download_active = true;
                    state_ = State::Downloading;
                    EXO_LOG("[RS485][MASTER][STATE] start download node=%u size=%lu\r\n",
                            (unsigned) kNodeIds[i], (unsigned long) nodes_[i].total_size);
                    request_next_chunk(kNodeIds[i], nodes_[i]);
                    return;
                }
            }
        }

        if (state_ == State::Downloading) {
            for (uint8_t i = 0U; i < kNodeCount; ++i) {
                if (!nodes_[i].download_active) {
                    continue;
                }
                if (nodes_[i].transfer_aborted) {
                    continue;
                }
                if (pending_[i].active) {
                    continue;
                }
                if (nodes_[i].receiver_credit == 0U) {
                    continue;
                }
                if ((nodes_[i].last_progress_ms != 0U) && ((now - nodes_[i].last_progress_ms) > kDownloadStallTimeoutMs)) {
                    nodes_[i].download_active = false;
                    nodes_[i].receiver_credit = 0U;
                    nodes_[i].recovery_request_mode = true;
                    EXO_LOG("[RS485][MASTER][REL] pause node=%u session=%lu reason=transport_timeout off=%lu\r\n",
                            (unsigned) kNodeIds[i],
                            (unsigned long) nodes_[i].session_id,
                            (unsigned long) nodes_[i].expected_offset);
                    continue;
                }
                if (!time_reached(now, nodes_[i].next_chunk_req_ms)) {
                    continue;
                }
                request_next_chunk(kNodeIds[i], nodes_[i]);
                return;
            }
        }
    }

    void request_next_chunk(uint8_t node_id, NodeState &node) {
        if (node.transfer_aborted || node.receiver_credit == 0U) {
            return;
        }
        if (node.expected_offset >= node.total_size) {
            node.receiver_credit = 0U;
            node.download_active = false;
            EXO_LOG("[RS485][MASTER][REL] source sent all chunks node=%u session=%lu waiting VERIFY_OK\r\n",
                    (unsigned) node_id,
                    (unsigned long) node.session_id);
            return;
        }
        const uint32_t remain = node.total_size - node.expected_offset;
        const uint16_t req_size = static_cast<uint16_t>(remain > kChunkPayloadMax ? kChunkPayloadMax : remain);
        const uint32_t current_chunk = node.expected_offset / static_cast<uint32_t>(kChunkPayloadMax);
        if (node.last_req_offset == node.expected_offset && node.last_req_size == req_size) {
            if (node.same_offset_retry_count < 0xFFU) {
                node.same_offset_retry_count++;
            }
        } else {
            node.same_offset_retry_count = 0U;
            node.last_req_offset = node.expected_offset;
            node.last_req_size = req_size;
        }
        if (node.same_offset_retry_count > kMaxSameOffsetRetries) {
            node.download_active = false;
            node.receiver_credit = 0U;
            node.recovery_request_mode = true;
            EXO_LOG("[RS485][MASTER][REL] pause node=%u session=%lu reason=same_offset_retry off=%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) node.session_id,
                    (unsigned long) node.expected_offset);
            return;
        }
        if (node.burst_supported && !node.recovery_request_mode) {
            DataRangeReqWire req{};
            req.session_id = node.session_id;
            req.start_chunk_index = current_chunk;
            req.chunk_size = kChunkPayloadMax;
            uint8_t want = node.receiver_credit == 0U ? 1U : node.receiver_credit;
            if (want > 16U) {
                want = 16U;
            }
            req.chunk_count = want;
            req.req_flags = kChunkReqNormal;
            req.expected_chunk_seq = node.expected_chunk_seq;
            node.last_req_wildcard_seq = false;
            EXO_LOG("[RS485][MASTER][RANGE_REQ] node=%u session=%lu start_chunk=%lu count=%u size=%u remain=%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) req.session_id,
                    (unsigned long) req.start_chunk_index,
                    (unsigned) req.chunk_count,
                    (unsigned) req.chunk_size,
                    (unsigned long) remain);
            (void)send_with_ack(node_id, MsgType::DataRangeReq, reinterpret_cast<const uint8_t *>(&req), sizeof(req));
        } else {
            DataChunkReqWire req{};
            req.session_id = node.session_id;
            req.offset = node.expected_offset;
            req.size = req_size;
            req.req_flags = node.recovery_request_mode ? kChunkReqRecovery : kChunkReqNormal;
            req.expected_chunk_seq = node.recovery_request_mode ? 0U : node.expected_chunk_seq;
            node.last_req_wildcard_seq = (req.expected_chunk_seq == 0U);
            EXO_LOG("[RS485][MASTER][CHUNK_REQ] node=%u session=%lu off=%lu size=%u remain=%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) req.session_id,
                    (unsigned long) req.offset,
                    (unsigned) req.size,
                    (unsigned long) remain);
            (void)send_with_ack(node_id, MsgType::DataChunkReq, reinterpret_cast<const uint8_t *>(&req), sizeof(req));
        }
        node.chunk_rerequest_count++;
    }

    void schedule_recovery(uint8_t node_id, NodeState &node, const char *reason) {
        const uint32_t now = HAL_GetTick();
        if (node.recovery_window_start_ms == 0U || (now - node.recovery_window_start_ms) > kRecoveryWindowMs) {
            node.recovery_window_start_ms = now;
            node.recovery_cycles_in_window = 0U;
        }
        if (node.recovery_cycles_in_window < 0xFFU) {
            node.recovery_cycles_in_window++;
        }
        if (node.recovery_cycles_in_window > kMaxRecoveryCyclesPerWindow) {
            node.download_active = false;
            node.receiver_credit = 0U;
            node.recovery_request_mode = true;
            EXO_LOG("[RS485][MASTER][REL] pause node=%u session=%lu reason=recovery_window off=%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) node.session_id,
                    (unsigned long) node.expected_offset);
            return;
        }
        uint32_t target = node.expected_offset;
        if (node.browser_acked_offset < target) {
            target = node.browser_acked_offset;
        }
        if (node.max_seen_offset > target) {
            const uint32_t rewind = node.max_seen_offset - target;
            if (rewind > kRecoveryWindowBytes) {
                target = node.max_seen_offset - kRecoveryWindowBytes;
            }
        }
        if (target != node.expected_offset) {
            EXO_LOG("[RS485][MASTER][RECOVER] node=%u session=%lu reason=%s off=%lu->%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) node.session_id,
                    reason,
                    (unsigned long) node.expected_offset,
                    (unsigned long) target);
        } else {
            EXO_LOG("[RS485][MASTER][RECOVER] node=%u session=%lu reason=%s off=%lu\r\n",
                    (unsigned) node_id,
                    (unsigned long) node.session_id,
                    reason,
                    (unsigned long) target);
        }
        node.recovery_events++;
        node.recovery_resume_offset = node.stream_offset;
        node.recovery_chunks_remaining = 1U;
        node.recovery_request_mode = true;
        node.expected_offset = target;
        const int idx = index_of_node(node_id);
        if (idx >= 0) {
            pending_[static_cast<uint8_t>(idx)].active = false;
        }
        clear_cached_chunk(node);
        node.next_chunk_req_ms = HAL_GetTick() + node.adaptive_delay_ms;
    }

    void abort_node_download(uint8_t node_id, NodeState &node, const char *reason) {
        if (node.transfer_aborted) {
            return;
        }
        node.transfer_aborted = true;
        node.recovery_request_mode = false;
        node.last_req_wildcard_seq = false;
        node.download_active = false;
        const int idx = index_of_node(node_id);
        if (idx >= 0) {
            pending_[static_cast<uint8_t>(idx)].active = false;
        }
        clear_cached_chunk(node);
        EXO_LOG("[RS485][MASTER][ABORT] node=%u session=%lu reason=%s sent=%lu/%lu retries=%lu bad=%u crc=%lu seq=%lu off=%lu len=%lu\r\n",
                (unsigned) node_id,
                (unsigned long) node.session_id,
                reason,
                (unsigned long) node.session_bytes_sent,
                (unsigned long) node.total_size,
                (unsigned long) node.chunk_rerequest_count,
                (unsigned) node.consecutive_bad_chunks,
                (unsigned long) node.crc_fail_count,
                (unsigned long) node.seq_mismatch_count,
                (unsigned long) node.offset_mismatch_count,
                (unsigned long) node.wire_len_fail_count);
    }

    void drain_rx_ring() {
        Frame frame{};
        while (rx_tail_ != rx_head_) {
            const uint8_t byte = rx_ring_[rx_tail_];
            rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxRingSize);
            if (parser_.feed(byte, frame)) {
                RS485_MASTER_LOG_XPORT("[RS485][MASTER][RX] src=%u dst=%u seq=%u type=%s len=%u\r\n",
                        (unsigned) frame.src_id,
                        (unsigned) frame.dst_id,
                        (unsigned) frame.sequence,
                        msg_type_str(frame.msg_type),
                        (unsigned) frame.payload_len);
                handle_frame(frame);
            }
        }
    }

    void handle_frame(const Frame &frame) {
        if (frame.dst_id != kMasterNodeId && frame.dst_id != kBroadcastNode) {
            RS485_MASTER_LOG_DISC("[RS485][MASTER][DROP] frame for dst=%u\r\n", (unsigned) frame.dst_id);
            return;
        }
        const int node_idx = index_of_node(frame.src_id);
        if (node_idx < 0) {
            RS485_MASTER_LOG_DISC("[RS485][MASTER][DROP] unknown src=%u\r\n", (unsigned) frame.src_id);
            return;
        }
        NodeState &node = nodes_[static_cast<uint8_t>(node_idx)];

        switch (frame.msg_type) {
            case MsgType::Ack:
            {
                MsgType pending_msg = MsgType::Error;
                bool had_pending = false;
                bool seq_match = false;
                const int pidx = index_of_node(frame.src_id);
                if (pidx >= 0) {
                    PendingTx &pp = pending_[static_cast<uint8_t>(pidx)];
                    had_pending = pp.active;
                    pending_msg = pp.msg;
                    seq_match = (pp.seq == frame.sequence);
                }
                /* CHUNK_REQ / RANGE_REQ use DataChunk as the transport-level ACK.
                 * Ignore plain ACK for this message type to avoid clearing inflight gating too early. */
                if (!(had_pending && seq_match &&
                      (pending_msg == MsgType::DataChunkReq || pending_msg == MsgType::DataRangeReq))) {
                    clear_pending(frame.src_id, frame.sequence);
                }
                if (had_pending && seq_match && pending_msg == MsgType::DiscoverReq && !node.discovered) {
                    node.discovered = true;
                    RS485_MASTER_LOG_DISC("[RS485][MASTER][DISC] discovered via ACK fallback node=%u\r\n",
                            (unsigned) frame.src_id);
                }
                if (had_pending && seq_match &&
                    (pending_msg == MsgType::DataChunkReq || pending_msg == MsgType::DataRangeReq)) {
                    RS485_MASTER_LOG_XPORT("[RS485][MASTER][ACK] ignore ctrl-ack for data req node=%u seq=%u\r\n",
                            (unsigned) frame.src_id,
                            (unsigned) frame.sequence);
                }
                if (node.await_start_ack && had_pending && seq_match && pending_msg == MsgType::StartRecord) {
                    node.await_start_ack = false;
                    EXO_LOG("[RS485][MASTER][ACK] start ack node=%u seq=%u\r\n",
                            (unsigned) frame.src_id, (unsigned) frame.sequence);
                }
                break;
            }
            case MsgType::Nack:
            {
                clear_pending(frame.src_id, frame.sequence);
                node.nack_count++;
                node.consecutive_bad_chunks++;
                uint8_t reason = 0U;
                if (frame.payload_len >= 1U) {
                    reason = frame.payload[0];
                }
                EXO_LOG("[RS485][MASTER][NACK] node=%u seq=%u reason=%u\r\n",
                        (unsigned) frame.src_id,
                        (unsigned) frame.sequence,
                        (unsigned) reason);
                schedule_recovery(frame.src_id, node, "peer_nack");
                break;
            }
            case MsgType::RecordStatus:
                send_ack(frame.src_id, frame.sequence);
                if (frame.payload_len == sizeof(exo::RecordDoneMessage)) {
                    exo::RecordDoneMessage done{};
                    memcpy(&done, frame.payload, sizeof(done));
                    node.done_received = true;
                    node.session_id = done.session_id;
                    node.actual_duration_ms = done.actual_duration_ms;
                    node.total_size = done.total_size;
                    node.payload_crc32 = done.payload_crc32;
                    node.expected_offset = 0U;
                    node.stream_offset = 0U;
                    node.recovery_resume_offset = 0U;
                    node.max_seen_offset = 0U;
                    node.browser_acked_offset = 0U;
                    node.last_browser_ack_ms = 0U;
                    node.browser_complete = false;
                    node.manifest_announced = false;
                    node.next_chunk_req_ms = HAL_GetTick() + kPostDoneDownloadDelayMs;
                    node.ble_busy_count = 0U;
                    node.chunk_rerequest_count = 0U;
                    node.chunk_accept_count = 0U;
                    node.session_bytes_sent = 0U;
                    node.session_start_ms = HAL_GetTick();
                    node.last_progress_ms = HAL_GetTick();
                    node.adaptive_delay_ms = kInterChunkDelayMs;
                    node.expected_chunk_seq = 1U;
                    node.recovery_request_mode = false;
                    node.last_req_wildcard_seq = false;
                    node.seq_resync_count = 0U;
                    node.last_req_offset = 0U;
                    node.last_req_size = 0U;
                    node.same_offset_retry_count = 0U;
                    node.recovery_window_start_ms = 0U;
                    node.recovery_cycles_in_window = 0U;
                    node.recovery_chunks_remaining = 0U;
                    node.recovery_events = 0U;
                    node.crc_fail_count = 0U;
                    node.seq_mismatch_count = 0U;
                    node.offset_mismatch_count = 0U;
                    node.wire_len_fail_count = 0U;
                    node.duplicate_count = 0U;
                    node.nack_count = 0U;
                    node.consecutive_bad_chunks = 0U;
                    node.transfer_aborted = false;
                    node.receiver_credit = 0U;
                    clear_cached_chunk(node);
                    last_record_done_ = done;
                    have_record_done_ = true;
                    EXO_LOG("[RS485][MASTER][DONE] node=%u session=%lu size=%lu crc=0x%08lX download_delay=%lums\r\n",
                            (unsigned) frame.src_id,
                            (unsigned long) done.session_id,
                            (unsigned long) done.total_size,
                            (unsigned long) done.payload_crc32,
                            (unsigned long) kPostDoneDownloadDelayMs);
                }
                break;
            case MsgType::DiscoverResp:
                send_ack(frame.src_id, frame.sequence);
                node.discovered = true;
                node.ready = (frame.payload_len >= 2U) ? (frame.payload[1] != 0U) : false;
                node.burst_supported = (frame.payload_len >= 3U) &&
                                       (frame.payload[2] >= kTransferProtocolVersionBurst);
                RS485_MASTER_LOG_DISC("[RS485][MASTER][DISC] discovered node=%u ready=%u burst=%u\r\n",
                        (unsigned) frame.src_id, (unsigned) node.ready, (unsigned) node.burst_supported);
                break;
            case MsgType::NodeIdResp:
                clear_pending(frame.src_id, frame.sequence);
                send_ack(frame.src_id, frame.sequence);
                if (frame.payload_len == sizeof(NodeIdRespWire)) {
                    NodeIdRespWire resp{};
                    memcpy(&resp, frame.payload, sizeof(resp));
                    EXO_LOG("[RS485][MASTER][NODE_ID] src=%u reported_id=%u status=%u\r\n",
                            (unsigned) frame.src_id,
                            (unsigned) resp.node_id,
                            (unsigned) resp.status);
                } else {
                    EXO_LOG("[RS485][MASTER][NODE_ID] bad payload len=%u\r\n",
                            (unsigned) frame.payload_len);
                }
                break;
            case MsgType::DataChunk:
                handle_data_chunk(node, frame);
                break;
            case MsgType::Error:
                send_ack(frame.src_id, frame.sequence);
                break;
            default:
                break;
        }
    }

    void handle_data_chunk(NodeState &node, const Frame &frame) {
        if (frame.payload_len < sizeof(DataChunkHdrWire)) {
            EXO_LOG("[RS485][MASTER][CHUNK] short payload len=%u\r\n", (unsigned) frame.payload_len);
            node.wire_len_fail_count++;
            node.consecutive_bad_chunks++;
            return;
        }
        DataChunkHdrWire hdr{};
        memcpy(&hdr, frame.payload, sizeof(hdr));
        if (hdr.session_id != node.session_id || hdr.offset != node.expected_offset) {
            if (hdr.session_id == node.session_id && hdr.offset < node.expected_offset) {
                node.duplicate_count++;
                EXO_LOG("[RS485][MASTER][CHUNK] duplicate off=%lu expect=%lu seq=%u\r\n",
                        (unsigned long) hdr.offset,
                        (unsigned long) node.expected_offset,
                        (unsigned) hdr.chunk_seq);
                return;
            }
            EXO_LOG("[RS485][MASTER][CHUNK] mismatch session=%lu/%lu off=%lu/%lu\r\n",
                    (unsigned long) hdr.session_id,
                    (unsigned long) node.session_id,
                    (unsigned long) hdr.offset,
                    (unsigned long) node.expected_offset);
            node.offset_mismatch_count++;
            node.consecutive_bad_chunks++;
            schedule_recovery(frame.src_id, node, "offset_mismatch");
            return;
        }
        const uint16_t expected_wire = static_cast<uint16_t>(sizeof(DataChunkHdrWire) + hdr.size);
        if (frame.payload_len != expected_wire) {
            EXO_LOG("[RS485][MASTER][CHUNK] wire len mismatch got=%u expect=%u\r\n",
                    (unsigned) frame.payload_len, (unsigned) expected_wire);
            node.wire_len_fail_count++;
            node.consecutive_bad_chunks++;
            schedule_recovery(frame.src_id, node, "wire_len");
            return;
        }
        const int pidx = index_of_node(frame.src_id);
        if (pidx < 0) {
            return;
        }
        PendingTx &pp = pending_[static_cast<uint8_t>(pidx)];
        if (pp.active && pp.seq == frame.sequence &&
            (pp.msg == MsgType::DataChunkReq || pp.msg == MsgType::DataRangeReq)) {
            clear_pending(frame.src_id, frame.sequence);
        }
        const uint8_t *chunk = &frame.payload[sizeof(DataChunkHdrWire)];
        const uint16_t calc_crc = crc16_ccitt(chunk, hdr.size);
        if (calc_crc != hdr.chunk_crc16) {
            EXO_LOG("[RS485][MASTER][CHUNK] crc fail off=%lu got=0x%04X calc=0x%04X\r\n",
                    (unsigned long) hdr.offset,
                    (unsigned) hdr.chunk_crc16,
                    (unsigned) calc_crc);
            node.crc_fail_count++;
            node.consecutive_bad_chunks++;
            schedule_recovery(frame.src_id, node, "chunk_crc");
            return;
        }
        if (!node.last_req_wildcard_seq && hdr.chunk_seq != node.expected_chunk_seq) {
            EXO_LOG("[RS485][MASTER][CHUNK] seq fail off=%lu got=%u expect=%u\r\n",
                    (unsigned long) hdr.offset,
                    (unsigned) hdr.chunk_seq,
                    (unsigned) node.expected_chunk_seq);
            node.seq_mismatch_count++;
            node.consecutive_bad_chunks++;
            schedule_recovery(frame.src_id, node, "chunk_seq");
            return;
        }
        if (node.last_req_wildcard_seq && hdr.chunk_seq != node.expected_chunk_seq) {
            EXO_LOG("[RS485][MASTER][RECOVER] seq drift node=%u session=%lu %u->%u off=%lu (telemetry)\r\n",
                    (unsigned) frame.src_id,
                    (unsigned long) node.session_id,
                    (unsigned) node.expected_chunk_seq,
                    (unsigned) hdr.chunk_seq,
                    (unsigned long) hdr.offset);
            node.expected_chunk_seq = hdr.chunk_seq;
            node.seq_resync_count++;
        }
        const bool was_recovery = node.recovery_request_mode;
        const uint32_t next_offset = hdr.offset + hdr.size;
        node.expected_offset = next_offset;
        if (!was_recovery) {
            node.stream_offset = next_offset;
        }
        if (node.max_seen_offset < next_offset) {
            node.max_seen_offset = next_offset;
        }
        node.expected_chunk_seq++;
        if (was_recovery && node.recovery_chunks_remaining > 0U) {
            node.recovery_chunks_remaining--;
        }
        if (!send_ble_session_chunk(node, frame.src_id, hdr.offset, frame.sequence, chunk, hdr.size)) {
            node.ble_busy_count++;
            adjust_adaptive_delay(node, true);
            cache_pending_chunk(node, frame.src_id, hdr.offset, frame.sequence, chunk, hdr.size);
            EXO_LOG("[RS485][MASTER][CHUNK] BLE send busy off=%lu size=%u, buffer and retry\r\n",
                    (unsigned long) hdr.offset,
                    (unsigned) hdr.size);
        } else {
            clear_cached_chunk(node);
        }
        if (was_recovery && node.recovery_chunks_remaining == 0U && node.ble_pending_active) {
            node.recovery_chunks_remaining = 1U;
            node.receiver_credit = 0U;
            node.next_chunk_req_ms = 0U;
            node.last_req_wildcard_seq = true;
            EXO_LOG("[RS485][MASTER][RECOVER] hold pending_ble node=%u session=%lu off=%lu\r\n",
                    (unsigned) frame.src_id,
                    (unsigned long) node.session_id,
                    (unsigned long) node.ble_pending_offset);
        } else if (was_recovery && node.recovery_chunks_remaining == 0U) {
            if (node.expected_offset < node.recovery_resume_offset) {
                node.expected_offset = node.recovery_resume_offset;
            }
            node.expected_chunk_seq = static_cast<uint16_t>((node.expected_offset / static_cast<uint32_t>(kChunkPayloadMax)) + 1U);
            EXO_LOG("[RS485][MASTER][RECOVER] exit node=%u session=%lu off=%lu\r\n",
                    (unsigned) frame.src_id,
                    (unsigned long) node.session_id,
                    (unsigned long) node.expected_offset);
            node.recovery_request_mode = false;
            node.last_req_wildcard_seq = false;
        } else if (was_recovery) {
            node.last_req_wildcard_seq = true;
        } else {
            node.last_req_wildcard_seq = false;
        }
        node.session_bytes_sent += hdr.size;
        node.chunk_accept_count++;
        if (node.burst_supported) {
            DataAckBitmapWire ack{};
            ack.session_id = node.session_id;
            ack.base_chunk_index = hdr.offset / static_cast<uint32_t>(kChunkPayloadMax);
            ack.bitmap = 0x0001U;
            ack.credit = node.receiver_credit;
            ack.flags = 0U;
            (void)send_frame(frame.src_id,
                             MsgType::DataAckBitmap,
                             reinterpret_cast<const uint8_t *>(&ack),
                             sizeof(ack),
                             false);
        }
        if (node.receiver_credit > 0U) {
            node.receiver_credit--;
        }
        node.consecutive_bad_chunks = 0U;
        node.last_progress_ms = HAL_GetTick();
        node.same_offset_retry_count = 0U;
        adjust_adaptive_delay(node, node.ble_pending_active);
        const uint32_t pct = (node.total_size > 0U)
                                 ? static_cast<uint32_t>((static_cast<uint64_t>(node.expected_offset) * 100ULL) / node.total_size)
                                 : 100U;
        EXO_LOG("[RS485][MASTER][CHUNK] accepted off=%lu size=%u next=%lu progress=%lu/%lu (%lu%%)\r\n",
                (unsigned long) hdr.offset,
                (unsigned) hdr.size,
                (unsigned long) node.expected_offset,
                (unsigned long) node.expected_offset,
                (unsigned long) node.total_size,
                (unsigned long) pct);
        node.next_chunk_req_ms = (node.receiver_credit > 0U) ? (HAL_GetTick() + node.adaptive_delay_ms) : 0U;
    }

    bool send_ble_session_chunk(const NodeState &node, uint16_t source_id, uint32_t offset, uint8_t sequence,
                                const uint8_t *payload, uint16_t payload_len) {
        if (ble_send_ == nullptr) {
            return false;
        }
        uint8_t packet[244] = {0U};
        exo::RecordReliableFrameHeader hdr{};
        hdr.command = exo::RecordCommand::ReliableFrame;
        hdr.proto_version = exo::kRecordReliableProtoVersion;
        hdr.magic = exo::kRecordReliableMagic;
        hdr.frame_type = static_cast<uint8_t>(exo::RecordReliableType::Chunk);
        hdr.session_id = node.session_id;
        hdr.source_id = source_id;
        hdr.chunk_index = offset / static_cast<uint32_t>(kChunkPayloadMax);
        hdr.byte_offset = offset;
        hdr.payload_len = payload_len;
        hdr.payload_crc16 = crc16_ccitt(payload, payload_len);
        hdr.flags = (((offset + payload_len) >= node.total_size) ? exo::kRecordFlagFinalChunk : 0U) |
                    (node.recovery_request_mode ? exo::kRecordFlagRetransmit : 0U);
        (void)sequence;
        const uint16_t total = static_cast<uint16_t>(sizeof(hdr) + payload_len);
        if (total > sizeof(packet)) {
            return false;
        }
        memcpy(packet, &hdr, sizeof(hdr));
        if (payload_len > 0U && payload != nullptr) {
            memcpy(packet + sizeof(hdr), payload, payload_len);
        }
        return ble_send_(packet, static_cast<uint8_t>(total));
    }

    void stream_session_bytes(NodeState &node, const uint8_t *data, uint16_t len) {
        uint16_t cursor = 0U;
        while (cursor < len) {
            const uint32_t logical_offset = node.expected_offset + cursor;
            if (logical_offset < sizeof(exo::SessionHeader)) {
                const uint16_t remaining_hdr = static_cast<uint16_t>(sizeof(exo::SessionHeader) - logical_offset);
                const uint16_t take = remaining_hdr < (len - cursor) ? remaining_hdr : static_cast<uint16_t>(len - cursor);
                memcpy(reinterpret_cast<uint8_t *>(&node.session_hdr) + logical_offset, &data[cursor], take);
                cursor += take;
                if ((logical_offset + take) == sizeof(exo::SessionHeader)) {
                    node.bno_remaining = node.session_hdr.bno85_payload_size;
                    node.icm_remaining = node.session_hdr.icm45686_payload_size;
                    node.stream_sensor = static_cast<uint8_t>(exo::BleSensorId::Bno85);
                }
                continue;
            }

            if (node.stream_sensor == static_cast<uint8_t>(exo::BleSensorId::Bno85) && node.bno_remaining == 0U) {
                node.stream_sensor = static_cast<uint8_t>(exo::BleSensorId::Icm45686);
            }
            if (node.stream_sensor == static_cast<uint8_t>(exo::BleSensorId::Icm45686) && node.icm_remaining == 0U) {
                return;
            }

            const uint16_t sample_size = (node.stream_sensor == static_cast<uint8_t>(exo::BleSensorId::Bno85))
                                             ? static_cast<uint16_t>(sizeof(exo::Bno85Sample))
                                             : static_cast<uint16_t>(sizeof(exo::Icm45686Sample));
            while (cursor < len) {
                const uint16_t need = static_cast<uint16_t>(sample_size - node.partial_len);
                const uint16_t take = (len - cursor) < need ? static_cast<uint16_t>(len - cursor) : need;
                memcpy(&node.partial[node.partial_len], &data[cursor], take);
                node.partial_len = static_cast<uint16_t>(node.partial_len + take);
                cursor = static_cast<uint16_t>(cursor + take);
                if (node.partial_len == sample_size) {
                    emit_ble_sample(node, node.stream_sensor, node.partial, sample_size);
                    node.partial_len = 0U;
                    if (node.stream_sensor == static_cast<uint8_t>(exo::BleSensorId::Bno85)) {
                        node.bno_remaining -= sample_size;
                        if (node.bno_remaining == 0U) {
                            node.stream_sensor = static_cast<uint8_t>(exo::BleSensorId::Icm45686);
                            break;
                        }
                    } else {
                        node.icm_remaining -= sample_size;
                    }
                }
            }
        }
    }

    void emit_ble_sample(const NodeState &node, uint8_t sensor_id, const uint8_t *payload, uint8_t payload_len) {
        if (ble_send_ == nullptr) {
            return;
        }
        uint8_t packet[156] = {0U};
        const uint8_t total = exo::ble_v2_pack(node.session_hdr.node_id,
                                               sensor_id,
                                               ble_sequence_++,
                                               HAL_GetTick(),
                                               payload,
                                               payload_len,
                                               packet,
                                               static_cast<uint8_t>(sizeof(packet)));
        if (total > 0U) {
            (void)ble_send_(packet, total);
        }
    }

    int index_of_node(uint8_t node_id) const {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (kNodeIds[i] == node_id) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    int index_of_session(uint32_t session_id) const {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].download_active && nodes_[i].session_id == session_id) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool send_with_ack(uint8_t dst_id, MsgType msg, const uint8_t *payload, uint16_t len) {
        const uint8_t seq = next_seq_++;
        if (!send_frame(dst_id, msg, payload, len, true, true, seq)) {
            RS485_MASTER_LOG_XPORT("[RS485][MASTER][TX] send_with_ack fail node=%u msg=%s seq=%u\r\n",
                    (unsigned) dst_id, msg_type_str(msg), (unsigned) seq);
            return false;
        }
        const int idx = index_of_node(dst_id);
        if (idx >= 0) {
            PendingTx &p = pending_[static_cast<uint8_t>(idx)];
            p.active = true;
            p.node_id = dst_id;
            p.seq = seq;
            p.msg = msg;
        p.last_tx_ms = 0U;
        p.ack_timeout_ms = (msg == MsgType::StartRecord) ? kStartRecordAckTimeoutMs : kAckTimeoutMs;
        p.retries = 0U;
        p.tx_started = false;
        }
        return true;
    }

    bool send_frame(uint8_t dst_id, MsgType msg, const uint8_t *payload, uint16_t len, bool cache_for_retry, bool has_forced_seq = false, uint8_t forced_seq = 0U) {
        Frame frame{};
        frame.msg_type = msg;
        frame.sequence = has_forced_seq ? forced_seq : next_seq_++;
        frame.src_id = kMasterNodeId;
        frame.dst_id = dst_id;
        frame.payload_len = len;
        if (len > 0U && payload != nullptr) {
            memcpy(frame.payload, payload, len);
        }
        uint8_t raw[kMaxFrameBytes] = {0U};
        uint16_t raw_len = 0U;
        if (!encode_frame(frame, raw, sizeof(raw), raw_len)) {
            RS485_MASTER_LOG_XPORT("[RS485][MASTER][TX] encode fail node=%u msg=%s seq=%u\r\n",
                    (unsigned) dst_id, msg_type_str(msg), (unsigned) frame.sequence);
            return false;
        }
        const bool queued = queue_raw_tx(raw, raw_len, tx_priority_for(msg), dst_id, frame.sequence, msg);
        RS485_MASTER_LOG_XPORT("[RS485][MASTER][TX] node=%u msg=%s seq=%u len=%u queued=%u\r\n",
                (unsigned) dst_id,
                msg_type_str(msg),
                (unsigned) frame.sequence,
                (unsigned) raw_len,
                (unsigned) (queued ? 1U : 0U));
        if (!queued) {
            return false;
        }
        if (cache_for_retry) {
            const int idx = index_of_node(dst_id);
            if (idx >= 0) {
                PendingTx &p = pending_[static_cast<uint8_t>(idx)];
                memcpy(p.frame_buf, raw, raw_len);
                p.frame_len = raw_len;
                p.seq = frame.sequence;
                p.msg = msg;
            }
        }
        return true;
    }

    void send_ack(uint8_t dst_id, uint8_t sequence) {
        Frame ack{};
        ack.msg_type = MsgType::Ack;
        ack.sequence = sequence;
        ack.src_id = kMasterNodeId;
        ack.dst_id = dst_id;
        ack.payload_len = 0U;
        uint8_t raw[kMaxFrameBytes] = {0U};
        uint16_t raw_len = 0U;
        if (encode_frame(ack, raw, sizeof(raw), raw_len)) {
            (void)queue_raw_tx(raw, raw_len, tx_priority_for(MsgType::Ack), dst_id, sequence, MsgType::Ack);
        }
    }

    void clear_pending(uint8_t node_id, uint8_t sequence) {
        const int idx = index_of_node(node_id);
        if (idx < 0) {
            return;
        }
        PendingTx &p = pending_[static_cast<uint8_t>(idx)];
        if (p.active && p.seq == sequence) {
            p.active = false;
            RS485_MASTER_LOG_XPORT("[RS485][MASTER][PENDING] clear node=%u seq=%u\r\n",
                    (unsigned) node_id, (unsigned) sequence);
        }
    }

    void reset_all() {
        next_seq_ = 1U;
        ble_sequence_ = 1U;
        tx_queue_head_ = 0U;
        tx_queue_tail_ = 0U;
        next_tx_allowed_ms_ = 0U;
        state_ = State::Idle;
        start_request_active_ = false;
        active_node_count_ = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i] = NodeState{};
            pending_[i] = PendingTx{};
        }
        for (uint8_t i = 0U; i < kTxQueueDepth; ++i) {
            tx_queue_[i] = TxQueueEntry{};
        }
    }

    void clear_runtime_state_only() {
        state_ = State::Idle;
        start_request_active_ = false;
        active_node_count_ = 0U;
        have_record_done_ = false;
        memset(&last_record_done_, 0, sizeof(last_record_done_));
        memset(&last_start_, 0, sizeof(last_start_));
        rx_head_ = 0U;
        rx_tail_ = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i].ready = false;
            nodes_[i].await_start_ack = false;
            nodes_[i].done_received = false;
            nodes_[i].download_active = false;
            nodes_[i].session_id = 0U;
            nodes_[i].actual_duration_ms = 0U;
            nodes_[i].expected_offset = 0U;
            nodes_[i].stream_offset = 0U;
            nodes_[i].recovery_resume_offset = 0U;
            nodes_[i].max_seen_offset = 0U;
            nodes_[i].total_size = 0U;
            nodes_[i].payload_crc32 = 0U;
            nodes_[i].expected_chunk_seq = 1U;
            nodes_[i].recovery_request_mode = false;
            nodes_[i].last_req_wildcard_seq = false;
            nodes_[i].seq_resync_count = 0U;
            nodes_[i].last_req_offset = 0U;
            nodes_[i].last_req_size = 0U;
            nodes_[i].same_offset_retry_count = 0U;
            nodes_[i].recovery_window_start_ms = 0U;
            nodes_[i].recovery_cycles_in_window = 0U;
            nodes_[i].recovery_chunks_remaining = 0U;
            nodes_[i].last_progress_ms = 0U;
            nodes_[i].recovery_events = 0U;
            nodes_[i].crc_fail_count = 0U;
            nodes_[i].seq_mismatch_count = 0U;
            nodes_[i].offset_mismatch_count = 0U;
            nodes_[i].wire_len_fail_count = 0U;
            nodes_[i].duplicate_count = 0U;
            nodes_[i].nack_count = 0U;
            nodes_[i].consecutive_bad_chunks = 0U;
            nodes_[i].transfer_aborted = false;
            nodes_[i].receiver_credit = 0U;
            nodes_[i].bno_remaining = 0U;
            nodes_[i].icm_remaining = 0U;
            nodes_[i].next_chunk_req_ms = 0U;
            nodes_[i].browser_acked_offset = 0U;
            nodes_[i].last_browser_ack_ms = 0U;
            nodes_[i].browser_complete = false;
            nodes_[i].manifest_announced = false;
            nodes_[i].stream_sensor = 0U;
            nodes_[i].partial_len = 0U;
            memset(nodes_[i].partial, 0, sizeof(nodes_[i].partial));
            clear_cached_chunk(nodes_[i]);
            memset(&nodes_[i].session_hdr, 0, sizeof(nodes_[i].session_hdr));
            pending_[i] = PendingTx{};
        }
    }

    bool send_reset_to_active_nodes_and_wait_ack() {
        uint8_t targets[kNodeCount] = {0U};
        uint8_t target_count = 0U;
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].discovered) {
                targets[target_count++] = kNodeIds[i];
            }
        }
        bool ok = true;
        for (uint8_t i = 0U; i < target_count; ++i) {
            if (!send_with_ack(targets[i], MsgType::ResetState, nullptr, 0U)) {
                ok = false;
            }
        }
        const uint32_t start_ms = HAL_GetTick();
        while ((HAL_GetTick() - start_ms) < (kAckTimeoutMs * (kMaxRetries + 1U) + 200U)) {
            drain_rx_ring();
            process_tx_queue();
            process_retries();
            bool any_target_pending = false;
            for (uint8_t i = 0U; i < target_count; ++i) {
                const int idx = index_of_node(targets[i]);
                if (idx >= 0 && pending_[static_cast<uint8_t>(idx)].active) {
                    any_target_pending = true;
                    break;
                }
            }
            if (!any_target_pending) {
                break;
            }
        }
        for (uint8_t i = 0U; i < target_count; ++i) {
            const int idx = index_of_node(targets[i]);
            if (idx >= 0 && pending_[static_cast<uint8_t>(idx)].active) {
                ok = false;
            }
        }
        return ok;
    }

    bool all_start_acked() const {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].await_start_ack) {
                return false;
            }
        }
        return true;
    }

    bool any_download_active() const {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (nodes_[i].download_active) {
                return true;
            }
        }
        return false;
    }

    bool any_pending_active() const {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            if (pending_[i].active) {
                return true;
            }
        }
        return false;
    }

    void abort_start_request() {
        for (uint8_t i = 0U; i < kNodeCount; ++i) {
            nodes_[i].await_start_ack = false;
            pending_[i].active = false;
        }
        state_ = State::Idle;
        start_request_active_ = false;
        active_node_count_ = 0U;
    }

    UART_HandleTypeDef &uart_;
    BleSendFn ble_send_ = nullptr;
    TxDoneFn tx_done_ = nullptr;
    FrameParser parser_{};
    uint8_t rx_ring_[kRxRingSize] = {0U};
    uint16_t rx_head_ = 0U;
    uint16_t rx_tail_ = 0U;
    exo::StartRecordMessage last_start_{};
    NodeState nodes_[kNodeCount]{};
    PendingTx pending_[kNodeCount]{};
    uint8_t next_seq_ = 1U;
    uint16_t ble_sequence_ = 1U;
    State state_ = State::Idle;
    exo::RecordDoneMessage last_record_done_{};
    bool have_record_done_ = false;
    bool start_request_active_ = false;
    uint8_t active_node_count_ = 0U;
    uint32_t last_closed_session_id_ = 0U;
    bool have_last_closed_session_ = false;
    bool transfer_hold_ = false;
    uint32_t last_rediscover_ms_ = 0U;
    uint32_t next_summary_ms_ = 0U;
    TxQueueEntry tx_queue_[kTxQueueDepth]{};
    uint8_t tx_queue_head_ = 0U;
    uint8_t tx_queue_tail_ = 0U;
    uint32_t next_tx_allowed_ms_ = 0U;
};

inline void MasterRecordingController::on_ble_chunk_ack(uint32_t session_id, uint16_t source_id, uint32_t next_offset) {
    const int idx = index_of_node(static_cast<uint8_t>(source_id));
    if (idx < 0) {
        return;
    }
    NodeState &node = nodes_[static_cast<uint8_t>(idx)];
    if (!node.download_active || node.session_id != session_id) {
        return;
    }
    if (next_offset > node.total_size) {
        next_offset = node.total_size;
    }
    node.browser_acked_offset = next_offset;
    node.last_browser_ack_ms = HAL_GetTick();
    if (next_offset >= node.total_size) {
        node.browser_complete = true;
        return;
    }
    if (next_offset < node.expected_offset) {
        const uint32_t rewind = node.expected_offset - next_offset;
        if (rewind > kRecoveryWindowBytes) {
            EXO_LOG("[RS485][MASTER][RECOVER] reject rewind node=%u session=%lu rewind=%lu limit=%u\r\n",
                    (unsigned) kNodeIds[static_cast<uint8_t>(idx)],
                    (unsigned long) node.session_id,
                    (unsigned long) rewind,
                    (unsigned) kRecoveryWindowBytes);
            abort_node_download(kNodeIds[static_cast<uint8_t>(idx)], node, "rewind_too_large");
            return;
        }
        schedule_recovery(kNodeIds[static_cast<uint8_t>(idx)], node, "ble_rewind");
    }
}

} // namespace exo::rs485_record

#endif

#ifndef RS485_RECORD_NODE_APP_H_
#define RS485_RECORD_NODE_APP_H_

#include <stdint.h>
#include <string.h>
#include <new>

extern "C" {
#include "usart.h"
}

#include "BLE_RECORD_PROTOCOL.h"
#include "EXO_LOGGER.h"
#include "NODE_RECORDING_APP.h"
#include "NODE_RUNTIME_CONFIG.h"
#include "RS485_FRAME_PROTOCOL.h"
#include "SESSION_TRANSFER.h"

namespace exo::rs485_record {

extern volatile uint8_t g_rs485_node_log_discovery;
extern volatile uint8_t g_rs485_node_log_transport;

#define RS485_NODE_LOG_DISC(...) do { if (g_rs485_node_log_discovery != 0U) { EXO_LOG(__VA_ARGS__); } } while (0)
#define RS485_NODE_LOG_XPORT(...) do { if (g_rs485_node_log_transport != 0U) { EXO_LOG(__VA_ARGS__); } } while (0)

class NodeRecordingResponder {
public:
    NodeRecordingResponder(UART_HandleTypeDef &uart, uint8_t node_id, exo::NodeRecordingApp *recording_app)
        : uart_(uart), node_id_(node_id), recording_app_(recording_app) {}

    void set_node_id(uint8_t node_id) {
        node_id_ = node_id;
    }

    void set_stream_enabled(bool enabled) { stream_enabled_ = enabled; }
    bool stream_enabled() const { return stream_enabled_; }
    void set_stream_interval_ms(uint8_t interval_ms) {
        stream_interval_ms_ = interval_ms == 0U ? 1U : interval_ms;
    }
    uint8_t stream_interval_ms() const { return stream_interval_ms_; }
    uint8_t node_id() const { return node_id_; }

    void begin() {
        if (!restart_rx_dma_idle()) {
            restart_rx_it();
        }
    }

    void process() {
        if (recording_app_ != nullptr) {
            recording_app_->process();
        }
        process_tx_queue();
        drain_rx_ring();
        maybe_stream_burst_chunks();
        maybe_report_done();
        drain_rx_ring();
        process_tx_queue();
    }

    void on_uart_rx_byte() {
        const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxRingSize);
        if (next == rx_tail_) {
            rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxRingSize);
            rx_overflow_count_++;
        }
        rx_ring_[rx_head_] = rx_it_byte_;
        rx_head_ = next;
        restart_rx_it();
    }

    void on_uart_rx_dma(const uint8_t *data, uint16_t len) {
        if (data == nullptr || len == 0U) {
            return;
        }
        for (uint16_t i = 0U; i < len; ++i) {
            const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxRingSize);
            if (next == rx_tail_) {
                rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxRingSize);
                rx_overflow_count_++;
            }
            rx_ring_[rx_head_] = data[i];
            rx_head_ = next;
        }
    }

    void on_uart_rx_idle_event(uint16_t len) {
        if (len > 0U) {
            on_uart_rx_dma(rx_dma_buf_, len);
        }
        (void)restart_rx_dma_idle();
    }

    void on_uart_error() {
        const uint32_t err = static_cast<uint32_t>(uart_.ErrorCode);
        const uint32_t now = HAL_GetTick();
        if (err != 0U && (last_uart_err_log_ms_ == 0U || (now - last_uart_err_log_ms_) >= 200U)) {
            EXO_LOG("[RS485][NODE%u][UART_ERR] err=0x%08lX, clear+restart RX\r\n",
                    (unsigned) node_id_,
                    (unsigned long) err);
            last_uart_err_log_ms_ = now;
        }
        (void)recover_uart_rx_it();
    }

private:
    enum class NackReason : uint8_t {
        InvalidPayload = 1U,
        UploadNotReady = 2U,
        SessionMismatch = 3U,
        RangeInvalid = 4U,
        ChunkSeqMismatch = 5U,
        ReadFail = 6U
    };

    struct ChunkCacheEntry {
        bool valid = false;
        uint32_t offset = 0U;
        uint16_t size = 0U;
        uint16_t seq = 0U;
        uint16_t crc16 = 0U;
        uint8_t data[512] = {0U};
    };

    struct TxQueueEntry {
        bool active = false;
        uint8_t raw[kMaxFrameBytes] = {0U};
        uint16_t raw_len = 0U;
        MsgType msg = MsgType::Error;
        uint8_t dst_id = 0U;
        uint8_t seq = 0U;
    };

    static void turnaround_guard_delay() {
        rs485_guard_us(kRxToTxGuardUs);
    }

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

    void restart_rx_it() {
        if (uart_rx_recovering_) {
            return;
        }
        const HAL_StatusTypeDef st = HAL_UART_Receive_IT(&uart_, &rx_it_byte_, 1U);
        if (st == HAL_BUSY || st == HAL_OK) {
            return;
        }
        const uint32_t err = static_cast<uint32_t>(uart_.ErrorCode);
        const HAL_StatusTypeDef retry = recover_uart_rx_it();
        RS485_NODE_LOG_XPORT("[RS485][NODE%u][RXIT] restart st=%d retry=%d err=0x%08lX\r\n",
                (unsigned) node_id_, (int) st, (int) retry, (unsigned long) err);
    }

    bool restart_rx_dma_idle() {
        if (uart_rx_recovering_) {
            return false;
        }
        const HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(&uart_, rx_dma_buf_, kRxDmaSize);
        if (st == HAL_OK || st == HAL_BUSY) {
            if (uart_.hdmarx != nullptr) {
                __HAL_DMA_DISABLE_IT(uart_.hdmarx, DMA_IT_HT);
            }
            use_dma_rx_ = true;
            return true;
        }
        use_dma_rx_ = false;
        return false;
    }

    HAL_StatusTypeDef recover_uart_rx_it() {
        if (uart_rx_recovering_) {
            return HAL_BUSY;
        }
        uart_rx_recovering_ = true;
        (void)HAL_UART_AbortReceive(&uart_);
        __HAL_UART_CLEAR_OREFLAG(&uart_);
        __HAL_UART_CLEAR_FEFLAG(&uart_);
        __HAL_UART_CLEAR_NEFLAG(&uart_);
        __HAL_UART_CLEAR_PEFLAG(&uart_);
#ifdef UART_RXDATA_FLUSH_REQUEST
        __HAL_UART_SEND_REQ(&uart_, UART_RXDATA_FLUSH_REQUEST);
#endif
        HAL_StatusTypeDef retry = HAL_ERROR;
        if (use_dma_rx_) {
            retry = HAL_UARTEx_ReceiveToIdle_DMA(&uart_, rx_dma_buf_, kRxDmaSize);
            if ((retry == HAL_OK || retry == HAL_BUSY) && uart_.hdmarx != nullptr) {
                __HAL_DMA_DISABLE_IT(uart_.hdmarx, DMA_IT_HT);
            }
        } else {
            retry = HAL_UART_Receive_IT(&uart_, &rx_it_byte_, 1U);
        }
        uart_rx_recovering_ = false;
        return retry;
    }

    void drain_rx_ring() {
        Frame frame{};
        while (rx_tail_ != rx_head_) {
            const uint8_t byte = rx_ring_[rx_tail_];
            rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxRingSize);
            if (parser_.feed(byte, frame)) {
                RS485_NODE_LOG_XPORT("[RS485][NODE%u][RX] src=%u dst=%u seq=%u type=%s len=%u\r\n",
                        (unsigned) node_id_,
                        (unsigned) frame.src_id,
                        (unsigned) frame.dst_id,
                        (unsigned) frame.sequence,
                        msg_type_str(frame.msg_type),
                        (unsigned) frame.payload_len);
                handle_frame(frame);
            }
        }
    }

    void process_tx_queue() {
        if (tx_queue_head_ == tx_queue_tail_) {
            return;
        }
        TxQueueEntry &entry = tx_queue_[tx_queue_tail_];
        if (!entry.active || entry.raw_len == 0U) {
            entry = TxQueueEntry{};
            tx_queue_tail_ = static_cast<uint8_t>((tx_queue_tail_ + 1U) % kTxQueueDepth);
            return;
        }
        rs485_guard_us(kRxToTxGuardUs);
        const HAL_StatusTypeDef tx = HAL_UART_Transmit(&uart_, entry.raw, entry.raw_len, kTxTimeoutMs);
        rs485_guard_us(kTxToRxGuardUs);
        restart_rx_it();
        RS485_NODE_LOG_XPORT("[RS485][NODE%u][TX] type=%s seq=%u dst=%u len=%u hal=%d q=%u\r\n",
                (unsigned) node_id_,
                msg_type_str(entry.msg),
                (unsigned) entry.seq,
                (unsigned) entry.dst_id,
                (unsigned) entry.raw_len,
                (int) tx,
                (unsigned) tx_queue_count());
        if (tx == HAL_OK) {
            entry = TxQueueEntry{};
            tx_queue_tail_ = static_cast<uint8_t>((tx_queue_tail_ + 1U) % kTxQueueDepth);
        }
    }

    void handle_frame(const Frame &frame) {
        if (frame.dst_id != node_id_ && frame.dst_id != kBroadcastNode) {
            RS485_NODE_LOG_DISC("[RS485][NODE%u][DROP] frame for dst=%u\r\n",
                    (unsigned) node_id_, (unsigned) frame.dst_id);
            return;
        }
        if (is_duplicate(frame)) {
            if (frame.msg_type == MsgType::DataChunkReq || frame.msg_type == MsgType::RequestChunk) {
                RS485_NODE_LOG_DISC("[RS485][NODE%u][DUP] src=%u seq=%u -> CHUNK resend\r\n",
                        (unsigned) node_id_, (unsigned) frame.src_id, (unsigned) frame.sequence);
                if (frame.msg_type == MsgType::DataChunkReq) {
                    on_chunk_req(frame);
                } else {
                    on_request_chunk(frame);
                }
                return;
            }
            RS485_NODE_LOG_DISC("[RS485][NODE%u][DUP] src=%u seq=%u -> ACK resend\r\n",
                    (unsigned) node_id_, (unsigned) frame.src_id, (unsigned) frame.sequence);
            if (tx_queue_count() >= static_cast<uint8_t>(kTxQueueDepth - 2U)) {
                RS485_NODE_LOG_DISC("[RS485][NODE%u][DUP] ack drop q=%u src=%u seq=%u\r\n",
                        (unsigned) node_id_,
                        (unsigned) tx_queue_count(),
                        (unsigned) frame.src_id,
                        (unsigned) frame.sequence);
                return;
            }
            send_ack(frame.src_id, frame.sequence);
            return;
        }

        switch (frame.msg_type) {
            case MsgType::DiscoverReq:
                on_discover_req(frame);
                break;
            case MsgType::StartRecord:
                on_start_record(frame);
                break;
            case MsgType::SetNodeIdReq:
                on_set_node_id(frame);
                break;
            case MsgType::GetNodeIdReq:
                on_get_node_id(frame);
                break;
            case MsgType::DataChunkReq:
                on_chunk_req(frame);
                break;
            case MsgType::DataRangeReq:
                on_range_req(frame);
                break;
            case MsgType::DataAckBitmap:
                on_ack_bitmap(frame);
                break;
            case MsgType::GetManifest:
                on_get_manifest(frame);
                break;
            case MsgType::RequestChunk:
                on_request_chunk(frame);
                break;
            case MsgType::PauseTransfer:
            case MsgType::ResumeTransfer:
                turnaround_guard_delay();
                send_ack(frame.src_id, frame.sequence);
                break;
            case MsgType::CancelTransfer:
                upload_started_ = false;
                reset_chunk_cache();
                turnaround_guard_delay();
                send_ack(frame.src_id, frame.sequence);
                break;
            case MsgType::TransferCompleteAck:
                on_transfer_complete(frame);
                break;
            case MsgType::CommitDone:
                on_commit_done(frame);
                break;
            case MsgType::ResetState:
                on_reset_state(frame);
                break;
            case MsgType::Ack:
                break;
            default:
                send_nack(frame.src_id, frame.sequence);
                break;
        }
    }

    void on_start_record(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != sizeof(exo::StartRecordMessage)) {
            EXO_LOG("[RS485][NODE%u][START] invalid app=%u len=%u expect=%u\r\n",
                    (unsigned) node_id_,
                    (unsigned) (recording_app_ != nullptr),
                    (unsigned) frame.payload_len,
                    (unsigned) sizeof(exo::StartRecordMessage));
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
            return;
        }
        exo::StartRecordMessage msg{};
        memcpy(&msg, frame.payload, sizeof(msg));
        const bool ok = recording_app_->start_recording(msg);
        EXO_LOG("[RS485][NODE%u][START] session=%lu result=%u\r\n",
                (unsigned) node_id_, (unsigned long) msg.session_id, (unsigned) ok);
        if (ok) {
            active_session_id_ = msg.session_id;
            upload_started_ = false;
            done_notified_ = false;
            pending_commit_wait_ = false;
            pending_commit_session_id_ = 0U;
            pending_commit_crc32_ = 0U;
            turnaround_guard_delay();
            send_ack(frame.src_id, frame.sequence);
        } else {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
        }
    }

    void on_discover_req(const Frame &frame) {
        RS485_NODE_LOG_DISC("[RS485][NODE%u][DISC] req from=%u seq=%u\r\n",
                (unsigned) node_id_, (unsigned) frame.src_id, (unsigned) frame.sequence);
        turnaround_guard_delay();
        send_ack(frame.src_id, frame.sequence);
        Frame resp{};
        resp.msg_type = MsgType::DiscoverResp;
        resp.sequence = next_seq_++;
        resp.src_id = node_id_;
        resp.dst_id = frame.src_id;
        resp.payload_len = 3U;
        resp.payload[0] = node_id_;
        resp.payload[1] = (recording_app_ != nullptr && recording_app_->can_start_recording()) ? 1U : 0U;
        resp.payload[2] = kTransferProtocolVersionBurst;
        (void)send_frame(resp);
    }

    void on_chunk_req(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != sizeof(DataChunkReqWire)) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] invalid app=%u len=%u expect=%u\r\n",
                    (unsigned) node_id_,
                    (unsigned) (recording_app_ != nullptr),
                    (unsigned) frame.payload_len,
                    (unsigned) sizeof(DataChunkReqWire));
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        DataChunkReqWire req{};
        memcpy(&req, frame.payload, sizeof(req));
        if (!ensure_upload_reader()) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] upload reader not ready\r\n", (unsigned) node_id_);
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::UploadNotReady);
            return;
        }
        if (req.session_id != active_session_id_) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] session mismatch req=%lu active=%lu\r\n",
                    (unsigned) node_id_,
                    (unsigned long) req.session_id,
                    (unsigned long) active_session_id_);
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::SessionMismatch);
            return;
        }
        if (req.size > kNodeBurstChunkSize) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] oversize req=%u max=%u\r\n",
                    (unsigned) node_id_, (unsigned) req.size, (unsigned) kNodeBurstChunkSize);
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::RangeInvalid);
            return;
        }
        if (req.size == 0U || req.offset > upload_total_size_ || (req.offset + req.size) > upload_total_size_) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] invalid range off=%lu size=%u total=%lu\r\n",
                    (unsigned) node_id_,
                    (unsigned long) req.offset,
                    (unsigned) req.size,
                    (unsigned long) upload_total_size_);
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::RangeInvalid);
            return;
        }
        uint16_t requested_chunk_seq = tx_chunk_seq_;
        if (req.expected_chunk_seq != 0U) {
            requested_chunk_seq = req.expected_chunk_seq;
        }
        const uint16_t previous_chunk_seq = static_cast<uint16_t>(tx_chunk_seq_ - 1U);
        if ((req.expected_chunk_seq != 0U) &&
            (req.expected_chunk_seq != tx_chunk_seq_) &&
            (req.expected_chunk_seq != previous_chunk_seq)) {
            EXO_LOG("[RS485][NODE%u][CHUNK_REQ] seq mismatch req=%u expect=%u\r\n",
                    (unsigned) node_id_,
                    (unsigned) req.expected_chunk_seq,
                    (unsigned) tx_chunk_seq_);
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::ChunkSeqMismatch);
            return;
        }

        const ChunkCacheEntry *cached = nullptr;
        if (req.expected_chunk_seq == 0U) {
            cached = find_cached_chunk_any_seq(req.offset, req.size);
        } else {
            cached = find_cached_chunk(req.offset, req.size, requested_chunk_seq);
        }
        uint16_t chunk_crc = 0U;
        uint16_t chunk_seq = requested_chunk_seq;
        const uint8_t *chunk_ptr = chunk_buf_;
        bool from_cache = false;
        if (cached != nullptr) {
            chunk_ptr = cached->data;
            chunk_crc = cached->crc16;
            chunk_seq = cached->seq;
            from_cache = true;
        } else {
            exo::SessionUploadReader *reader = reinterpret_cast<exo::SessionUploadReader *>(upload_reader_storage_);
            if (!reader->read(req.offset, chunk_buf_, req.size)) {
                EXO_LOG("[RS485][NODE%u][CHUNK_REQ] read fail off=%lu size=%u\r\n",
                        (unsigned) node_id_, (unsigned long) req.offset, (unsigned) req.size);
                turnaround_guard_delay();
                send_nack(frame.src_id, frame.sequence, NackReason::ReadFail);
                return;
            }
            chunk_crc = crc16_ccitt(chunk_buf_, req.size);
            chunk_seq = tx_chunk_seq_;
        }
        EXO_LOG("[RS485][NODE%u][CHUNK_REQ] session=%lu off=%lu size=%u\r\n",
                (unsigned) node_id_, (unsigned long) req.session_id, (unsigned long) req.offset, (unsigned) req.size);
        DataChunkHdrWire hdr{};
        hdr.session_id = req.session_id;
        hdr.offset = req.offset;
        hdr.size = req.size;
        hdr.chunk_crc16 = chunk_crc;
        hdr.chunk_seq = chunk_seq;
        hdr.origin_flags = from_cache ? kChunkOriginCacheHit : kChunkOriginFreshRead;
        uint8_t payload[sizeof(DataChunkHdrWire) + kNodeBurstChunkSize] = {0U};
        memcpy(payload, &hdr, sizeof(hdr));
        memcpy(payload + sizeof(hdr), chunk_ptr, req.size);

        Frame out{};
        out.msg_type = MsgType::DataChunk;
        out.sequence = frame.sequence;
        out.src_id = node_id_;
        out.dst_id = frame.src_id;
        out.payload_len = static_cast<uint16_t>(sizeof(DataChunkHdrWire) + req.size);
        memcpy(out.payload, payload, out.payload_len);
        turnaround_guard_delay();
        send_frame(out);
        if (!from_cache) {
            cache_chunk(req.offset, req.size, chunk_seq, chunk_crc, chunk_ptr);
            tx_chunk_seq_++;
        }
    }

    void on_get_manifest(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != sizeof(ManifestReqWire)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        ManifestReqWire req{};
        memcpy(&req, frame.payload, sizeof(req));
        if (!recording_app_->session_ready()) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::UploadNotReady);
            return;
        }
        const exo::RecordDoneMessage done = recording_app_->make_record_done();
        ManifestWire manifest{};
        manifest.session_id = done.session_id;
        manifest.source_id = node_id_;
        manifest.total_size = done.total_size;
        manifest.chunk_size = kNodeBurstChunkSize;
        manifest.total_chunks = static_cast<uint16_t>((done.total_size + kNodeBurstChunkSize - 1U) / kNodeBurstChunkSize);
        manifest.payload_crc32 = done.payload_crc32;
        if (req.session_id != 0U && req.session_id != done.session_id) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::SessionMismatch);
            return;
        }
        Frame out{};
        out.msg_type = MsgType::Manifest;
        out.sequence = frame.sequence;
        out.src_id = node_id_;
        out.dst_id = frame.src_id;
        out.payload_len = sizeof(manifest);
        memcpy(out.payload, &manifest, sizeof(manifest));
        turnaround_guard_delay();
        (void)send_frame(out);
    }

    void on_request_chunk(const Frame &frame) {
        if (frame.payload_len != sizeof(RequestChunkWire)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        RequestChunkWire req{};
        memcpy(&req, frame.payload, sizeof(req));
        DataChunkReqWire legacy{};
        legacy.session_id = req.session_id;
        legacy.offset = req.chunk_index * static_cast<uint32_t>(kNodeBurstChunkSize);
        legacy.size = req.size == 0U ? kNodeBurstChunkSize : req.size;
        if (legacy.size > kNodeBurstChunkSize) {
            legacy.size = kNodeBurstChunkSize;
        }
        legacy.req_flags = req.req_flags;
        legacy.expected_chunk_seq = 0U;
        Frame mapped = frame;
        mapped.msg_type = MsgType::DataChunkReq;
        mapped.payload_len = sizeof(legacy);
        memcpy(mapped.payload, &legacy, sizeof(legacy));
        on_chunk_req(mapped);
    }

    void on_range_req(const Frame &frame) {
        if (frame.payload_len != sizeof(DataRangeReqWire)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        DataRangeReqWire req{};
        memcpy(&req, frame.payload, sizeof(req));
        if (req.chunk_count == 0U) {
            req.chunk_count = 1U;
        }
        if (req.chunk_count > kBurstWindowMax) {
            req.chunk_count = kBurstWindowMax;
        }
        if (req.chunk_size == 0U || req.chunk_size > kNodeBurstChunkSize) {
            req.chunk_size = kNodeBurstChunkSize;
        }
        burst_active_ = true;
        burst_session_id_ = req.session_id;
        burst_next_chunk_index_ = req.start_chunk_index;
        burst_chunk_size_ = req.chunk_size;
        burst_remaining_ = req.chunk_count;
        burst_last_src_ = frame.src_id;
        burst_credit_ = req.chunk_count;
        turnaround_guard_delay();
        send_ack(frame.src_id, frame.sequence);
    }

    void on_ack_bitmap(const Frame &frame) {
        if (frame.payload_len < sizeof(DataAckBitmapWire)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        DataAckBitmapWire ack{};
        memcpy(&ack, frame.payload, sizeof(ack));
        burst_credit_ = ack.credit;
        turnaround_guard_delay();
        send_ack(frame.src_id, frame.sequence);
    }

    void on_set_node_id(const Frame &frame) {
        if (frame.payload_len != sizeof(SetNodeIdWire)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
            return;
        }
        SetNodeIdWire req{};
        memcpy(&req, frame.payload, sizeof(req));
        NodeIdRespWire resp{};
        if (!exo::node_runtime_config::is_valid_node_id(req.new_id) ||
            recording_app_ == nullptr ||
            !exo::node_runtime_config::store_node_id(req.new_id)) {
            resp.node_id = node_id_;
            resp.status = 1U;
            send_node_id_resp(frame.src_id, frame.sequence, resp);
            return;
        }
        resp.node_id = req.new_id;
        resp.status = 0U;
        send_node_id_resp(frame.src_id, frame.sequence, resp);
        EXO_LOG("[RS485][NODE%u][NODE_ID] stored new_id=%u apply_on_reboot=1\r\n",
                (unsigned) node_id_, (unsigned) req.new_id);
    }

    void on_get_node_id(const Frame &frame) {
        if (frame.payload_len != 0U) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
            return;
        }
        NodeIdRespWire resp{};
        resp.node_id = node_id_;
        resp.status = 0U;
        send_node_id_resp(frame.src_id, frame.sequence, resp);
    }

    void on_transfer_complete(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != sizeof(exo::SessionCompleteAckMessage)) {
            EXO_LOG("[RS485][NODE%u][XFER_DONE] invalid app=%u len=%u expect=%u\r\n",
                    (unsigned) node_id_,
                    (unsigned) (recording_app_ != nullptr),
                    (unsigned) frame.payload_len,
                    (unsigned) sizeof(exo::SessionCompleteAckMessage));
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
            return;
        }
        exo::SessionCompleteAckMessage ack{};
        memcpy(&ack, frame.payload, sizeof(ack));
        const bool ok = recording_app_->transfer_complete();
        pending_commit_wait_ = ok;
        pending_commit_session_id_ = ack.session_id;
        pending_commit_crc32_ = ack.payload_crc32;
        EXO_LOG("[RS485][NODE%u][XFER_DONE] session=%lu transfer_complete=%u wait_commit=%u\r\n",
                (unsigned) node_id_, (unsigned long) ack.session_id, (unsigned) ok, (unsigned) pending_commit_wait_);
        if (ok) {
            turnaround_guard_delay();
            send_ack(frame.src_id, frame.sequence);
        } else {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
        }
    }

    void on_reset_state(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != 0U) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
            return;
        }
        const bool ok = recording_app_->reset_to_idle_and_erase();
        EXO_LOG("[RS485][NODE%u][RESET] result=%u\r\n",
                (unsigned) node_id_, (unsigned) ok);
        if (ok) {
            upload_started_ = false;
            done_notified_ = false;
            active_session_id_ = 0U;
            upload_total_size_ = 0U;
            tx_chunk_seq_ = 1U;
            pending_commit_wait_ = false;
            pending_commit_session_id_ = 0U;
            pending_commit_crc32_ = 0U;
            reset_chunk_cache();
            turnaround_guard_delay();
            send_ack(frame.src_id, frame.sequence);
        } else {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence);
        }
    }

    bool ensure_upload_reader() {
        if (recording_app_ == nullptr) {
            return false;
        }
        if (!upload_started_) {
            if (!recording_app_->session_ready() || !recording_app_->begin_upload()) {
                EXO_LOG("[RS485][NODE%u][UPLOAD] session not ready or begin_upload fail\r\n", (unsigned) node_id_);
                return false;
            }
            new (upload_reader_storage_) exo::SessionUploadReader(recording_app_->make_upload_reader());
            upload_started_ = true;
            upload_total_size_ = recording_app_->make_record_done().total_size;
            tx_chunk_seq_ = 1U;
            reset_chunk_cache();
            EXO_LOG("[RS485][NODE%u][UPLOAD] reader ready\r\n", (unsigned) node_id_);
        }
        return true;
    }

    void reset_chunk_cache() {
        cache_next_idx_ = 0U;
        for (uint8_t i = 0U; i < kChunkCacheDepth; ++i) {
            chunk_cache_[i] = ChunkCacheEntry{};
        }
    }

    const ChunkCacheEntry *find_cached_chunk(uint32_t offset, uint16_t size, uint16_t seq) const {
        for (uint8_t i = 0U; i < kChunkCacheDepth; ++i) {
            const ChunkCacheEntry &entry = chunk_cache_[i];
            if (!entry.valid) {
                continue;
            }
            if (entry.offset == offset && entry.size == size && entry.seq == seq) {
                return &entry;
            }
        }
        return nullptr;
    }

    const ChunkCacheEntry *find_cached_chunk_any_seq(uint32_t offset, uint16_t size) const {
        for (uint8_t i = 0U; i < kChunkCacheDepth; ++i) {
            const ChunkCacheEntry &entry = chunk_cache_[i];
            if (!entry.valid) {
                continue;
            }
            if (entry.offset == offset && entry.size == size) {
                return &entry;
            }
        }
        return nullptr;
    }

    void cache_chunk(uint32_t offset, uint16_t size, uint16_t seq, uint16_t crc16, const uint8_t *data) {
        if (data == nullptr || size > kNodeBurstChunkSize) {
            return;
        }
        ChunkCacheEntry &entry = chunk_cache_[cache_next_idx_];
        entry.valid = true;
        entry.offset = offset;
        entry.size = size;
        entry.seq = seq;
        entry.crc16 = crc16;
        memcpy(entry.data, data, size);
        cache_next_idx_ = static_cast<uint8_t>((cache_next_idx_ + 1U) % kChunkCacheDepth);
    }

    void maybe_report_done() {
        if (recording_app_ == nullptr || done_notified_) {
            return;
        }
        if (!recording_app_->session_ready()) {
            return;
        }
        const exo::RecordDoneMessage done = recording_app_->make_record_done();
        Frame out{};
        out.msg_type = MsgType::RecordStatus;
        out.sequence = next_seq_++;
        out.src_id = node_id_;
        out.dst_id = 0U;
        out.payload_len = sizeof(done);
        memcpy(out.payload, &done, sizeof(done));
        if (send_frame(out)) {
            done_notified_ = true;
            EXO_LOG("[RS485][NODE%u][DONE] session=%lu size=%lu crc=0x%08lX\r\n",
                    (unsigned) node_id_,
                    (unsigned long) done.session_id,
                    (unsigned long) done.total_size,
                    (unsigned long) done.payload_crc32);
        }
    }

    void on_commit_done(const Frame &frame) {
        if (recording_app_ == nullptr || frame.payload_len != sizeof(exo::RecordReliableVerifyPayload)) {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::InvalidPayload);
            return;
        }
        exo::RecordReliableVerifyPayload verify{};
        memcpy(&verify, frame.payload, sizeof(verify));
        const bool match = pending_commit_wait_ &&
                           (verify.session_id == pending_commit_session_id_) &&
                           (verify.file_crc32 == pending_commit_crc32_);
        const bool ok = match && recording_app_->acknowledge_and_erase();
        EXO_LOG("[RS485][NODE%u][COMMIT] session=%lu crc=0x%08lX match=%u erase=%u\r\n",
                (unsigned) node_id_,
                (unsigned long) verify.session_id,
                (unsigned long) verify.file_crc32,
                (unsigned) match,
                (unsigned) ok);
        if (ok) {
            upload_started_ = false;
            done_notified_ = false;
            active_session_id_ = 0U;
            upload_total_size_ = 0U;
            tx_chunk_seq_ = 1U;
            pending_commit_wait_ = false;
            pending_commit_session_id_ = 0U;
            pending_commit_crc32_ = 0U;
            reset_chunk_cache();
            turnaround_guard_delay();
            send_ack(frame.src_id, frame.sequence);
        } else {
            turnaround_guard_delay();
            send_nack(frame.src_id, frame.sequence, NackReason::SessionMismatch);
        }
    }

    void maybe_stream_burst_chunks() {
        if (!burst_active_ || burst_remaining_ == 0U) {
            return;
        }
        if (tx_queue_count() > (kTxQueueDepth - 4U) || burst_credit_ == 0U) {
            return;
        }

        DataChunkReqWire req{};
        req.session_id = burst_session_id_;
        req.offset = burst_next_chunk_index_ * static_cast<uint32_t>(burst_chunk_size_);
        req.size = burst_chunk_size_;
        req.req_flags = kChunkReqNormal;
        req.expected_chunk_seq = 0U;

        Frame mapped{};
        mapped.msg_type = MsgType::DataChunkReq;
        mapped.sequence = next_seq_++;
        mapped.src_id = burst_last_src_;
        mapped.dst_id = node_id_;
        mapped.payload_len = sizeof(req);
        memcpy(mapped.payload, &req, sizeof(req));
        on_chunk_req(mapped);

        burst_next_chunk_index_++;
        burst_remaining_--;
        burst_credit_--;
        if (burst_remaining_ == 0U) {
            burst_active_ = false;
        }
    }

    bool send_frame(const Frame &frame) {
        uint8_t raw[kMaxFrameBytes] = {0U};
        uint16_t raw_len = 0U;
        if (!encode_frame(frame, raw, sizeof(raw), raw_len)) {
            RS485_NODE_LOG_XPORT("[RS485][NODE%u][TX] encode fail type=%s seq=%u\r\n",
                    (unsigned) node_id_, msg_type_str(frame.msg_type), (unsigned) frame.sequence);
            return false;
        }
        const bool queued = queue_raw_tx(raw, raw_len, frame.msg_type, frame.dst_id, frame.sequence);
        RS485_NODE_LOG_XPORT("[RS485][NODE%u][TXQ] type=%s seq=%u dst=%u len=%u queued=%u q=%u\r\n",
                (unsigned) node_id_,
                msg_type_str(frame.msg_type),
                (unsigned) frame.sequence,
                (unsigned) frame.dst_id,
                (unsigned) raw_len,
                (unsigned) (queued ? 1U : 0U),
                (unsigned) tx_queue_count());
        return queued;
    }

    bool queue_raw_tx(const uint8_t *raw, uint16_t raw_len, MsgType msg, uint8_t dst_id, uint8_t seq) {
        const uint8_t next = static_cast<uint8_t>((tx_queue_head_ + 1U) % kTxQueueDepth);
        if (next == tx_queue_tail_) {
            EXO_LOG("[RS485][NODE%u][TXQ] overflow msg=%s seq=%u len=%u\r\n",
                    (unsigned) node_id_,
                    msg_type_str(msg),
                    (unsigned) seq,
                    (unsigned) raw_len);
            return false;
        }
        TxQueueEntry &entry = tx_queue_[tx_queue_head_];
        entry = TxQueueEntry{};
        entry.active = true;
        entry.raw_len = raw_len;
        entry.msg = msg;
        entry.dst_id = dst_id;
        entry.seq = seq;
        memcpy(entry.raw, raw, raw_len);
        tx_queue_head_ = next;
        return true;
    }

    uint8_t tx_queue_count() const {
        if (tx_queue_head_ >= tx_queue_tail_) {
            return static_cast<uint8_t>(tx_queue_head_ - tx_queue_tail_);
        }
        return static_cast<uint8_t>(kTxQueueDepth - tx_queue_tail_ + tx_queue_head_);
    }

    static void rs485_guard_us(uint32_t us) {
        const uint32_t loops = ((SystemCoreClock / 1000000U) * us) / 6U;
        for (volatile uint32_t i = 0U; i < loops; ++i) {
            __NOP();
        }
    }

    void send_ack(uint8_t dst_id, uint8_t sequence) {
        Frame ack{};
        ack.msg_type = MsgType::Ack;
        ack.sequence = sequence;
        ack.src_id = node_id_;
        ack.dst_id = dst_id;
        ack.payload_len = 0U;
        (void)send_frame(ack);
    }

    void send_nack(uint8_t dst_id, uint8_t sequence, NackReason reason = NackReason::InvalidPayload) {
        Frame nack{};
        nack.msg_type = MsgType::Nack;
        nack.sequence = sequence;
        nack.src_id = node_id_;
        nack.dst_id = dst_id;
        nack.payload_len = 1U;
        nack.payload[0] = static_cast<uint8_t>(reason);
        (void)send_frame(nack);
    }

    void send_node_id_resp(uint8_t dst_id, uint8_t sequence, const NodeIdRespWire &resp) {
        Frame out{};
        out.msg_type = MsgType::NodeIdResp;
        out.sequence = sequence;
        out.src_id = node_id_;
        out.dst_id = dst_id;
        out.payload_len = sizeof(resp);
        memcpy(out.payload, &resp, sizeof(resp));
        turnaround_guard_delay();
        (void)send_frame(out);
    }

    bool is_duplicate(const Frame &frame) {
        if (frame.msg_type == MsgType::Ack || frame.msg_type == MsgType::Nack) {
            return false;
        }
        if (last_src_ == frame.src_id && last_seq_ == frame.sequence) {
            return true;
        }
        last_src_ = frame.src_id;
        last_seq_ = frame.sequence;
        return false;
    }

    static constexpr uint32_t kRs485Baud = 921600U;
    static constexpr uint32_t kCharTimeUs = 11U;
    static constexpr uint32_t kTxToRxGuardUs = 25U;
    static constexpr uint32_t kRxToTxGuardUs = 25U;
    static constexpr uint32_t kTxTimeoutMs = 100U;
    static constexpr uint16_t kNodeChunkSize = 180U;
    static constexpr uint16_t kNodeBurstChunkSize = 512U;
    static constexpr uint8_t kBurstWindowMax = 16U;
    static constexpr uint16_t kRxRingSize = 512U;
    static constexpr uint16_t kRxDmaSize = 256U;
    static constexpr uint8_t kChunkCacheDepth = 4U;
    static constexpr uint8_t kTxQueueDepth = 12U;

    UART_HandleTypeDef &uart_;
    uint8_t node_id_ = 0U;
    exo::NodeRecordingApp *recording_app_ = nullptr;
    bool stream_enabled_ = false;
    uint8_t stream_interval_ms_ = 20U;
    FrameParser parser_{};
    uint8_t next_seq_ = 1U;
    uint8_t last_src_ = 0U;
    uint8_t last_seq_ = 0U;
    uint8_t rx_it_byte_ = 0U;
    bool use_dma_rx_ = false;
    bool uart_rx_recovering_ = false;
    uint32_t last_uart_err_log_ms_ = 0U;
    volatile uint16_t rx_head_ = 0U;
    volatile uint16_t rx_tail_ = 0U;
    volatile uint32_t rx_overflow_count_ = 0U;
    uint8_t rx_ring_[kRxRingSize] = {0U};
    uint8_t rx_dma_buf_[kRxDmaSize] = {0U};
    uint8_t chunk_buf_[kNodeBurstChunkSize] = {0U};
    alignas(exo::SessionUploadReader) uint8_t upload_reader_storage_[sizeof(exo::SessionUploadReader)] = {0U};
    bool upload_started_ = false;
    bool done_notified_ = false;
    uint32_t active_session_id_ = 0U;
    uint32_t upload_total_size_ = 0U;
    uint16_t tx_chunk_seq_ = 1U;
    ChunkCacheEntry chunk_cache_[kChunkCacheDepth]{};
    uint8_t cache_next_idx_ = 0U;
    bool burst_active_ = false;
    uint32_t burst_session_id_ = 0U;
    uint32_t burst_next_chunk_index_ = 0U;
    uint16_t burst_chunk_size_ = kNodeBurstChunkSize;
    uint8_t burst_remaining_ = 0U;
    uint8_t burst_credit_ = 0U;
    uint8_t burst_last_src_ = 0U;
    bool pending_commit_wait_ = false;
    uint32_t pending_commit_session_id_ = 0U;
    uint32_t pending_commit_crc32_ = 0U;
    TxQueueEntry tx_queue_[kTxQueueDepth]{};
    uint8_t tx_queue_head_ = 0U;
    uint8_t tx_queue_tail_ = 0U;
};

} // namespace exo::rs485_record

#endif

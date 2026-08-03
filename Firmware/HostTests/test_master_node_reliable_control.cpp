#include <cassert>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

#include "MASTER_NODE_RELIABLE_CONTROL.h"

struct FakeTransport {
    int failures_remaining = 1;
    std::vector<uint8_t> frame;
    uint8_t node = 0U;
    static bool send(void *context, uint8_t node, const uint8_t *frame,
            uint16_t length)
    {
        auto &self = *static_cast<FakeTransport *>(context);
        self.node = node;
        if (self.failures_remaining > 0) {
            --self.failures_remaining;
            return false;
        }
        self.frame.assign(frame, frame + length);
        return true;
    }
};

static exo::RecordReliableFrameHeader header_of(
        const std::vector<uint8_t> &frame)
{
    assert(frame.size() >= sizeof(exo::RecordReliableFrameHeader));
    exo::RecordReliableFrameHeader header{};
    std::memcpy(&header, frame.data(), sizeof(header));
    return header;
}

template<typename T>
static T body_of(const std::vector<uint8_t> &frame)
{
    T body{};
    assert(frame.size() == sizeof(exo::RecordReliableFrameHeader) + sizeof(T));
    std::memcpy(&body, frame.data() + sizeof(exo::RecordReliableFrameHeader),
            sizeof(T));
    return body;
}

int main()
{
    FakeTransport transport;
    exo::MasterNodeReliableControl control(&FakeTransport::send, &transport);
    exo::RecordDoneMessage done{};
    done.command = exo::RecordCommand::RecordDone;
    done.node_id = 3U;
    done.session_id = 42U;
    done.total_size = 1000U;
    done.payload_crc32 = 0x12345678U;

    assert(control.begin(done));
    assert(control.pending());
    assert(!control.service(100U));
    assert(control.pending());
    assert(!control.service(110U));
    assert(control.service(120U));
    assert(!control.pending());
    assert(transport.node == 3U);

    auto header = header_of(transport.frame);
    assert(header.command == exo::RecordCommand::ReliableFrame);
    assert(header.proto_version == 6U);
    assert(header.magic == exo::kRecordReliableMagic);
    assert(header.frame_type ==
            static_cast<uint8_t>(exo::RecordReliableType::ManifestAck));
    auto manifest_ack =
            body_of<exo::RecordReliableManifestAckPayload>(transport.frame);
    assert(manifest_ack.source_id == 3U);
    assert(manifest_ack.session_id == 42U);
    assert(manifest_ack.accepted_chunk_size == 180U);
    assert(manifest_ack.credit == 8U);
    assert(header.payload_crc16 ==
            exo::MasterNodeReliableControl::crc16_ccitt(
                    reinterpret_cast<const uint8_t *>(&manifest_ack),
                    sizeof(manifest_ack)));

    assert(control.ack_window(5U, 9U));
    assert(control.nack_range(4U, 2U));
    assert(control.pending_type() == exo::RecordReliableType::NackRange);
    assert(control.service(200U));
    header = header_of(transport.frame);
    assert(header.frame_type ==
            static_cast<uint8_t>(exo::RecordReliableType::NackRange));
    auto nack = body_of<exo::RecordReliableNackRangePayload>(transport.frame);
    assert(nack.first_chunk_index == 4U);
    assert(nack.chunk_count == 2U);

    assert(control.ack_window(6U, 8U));
    assert(control.ack_window(7U, 8U));
    assert(control.service(230U));
    auto ack = body_of<exo::RecordReliableAckWindowPayload>(transport.frame);
    assert(ack.next_chunk_index == 7U);
    assert(ack.credit == 8U);

    assert(!control.verify_ok(0xDEADBEEFU));
    assert(control.verify_ok(done.payload_crc32));
    assert(control.service(260U));
    header = header_of(transport.frame);
    assert(header.frame_type ==
            static_cast<uint8_t>(exo::RecordReliableType::VerifyOk));
    auto verify = body_of<exo::RecordReliableVerifyPayload>(transport.frame);
    assert(verify.file_crc32 == done.payload_crc32);

    std::cout << "master node reliable control tests passed\n";
    return 0;
}

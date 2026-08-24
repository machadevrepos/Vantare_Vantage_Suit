#include <stdint.h>
#include <string.h>

#include <exo/recording/node_live_sample_queue.h>

static bool same_bytes(const uint8_t *lhs, const uint8_t *rhs, uint8_t size)
{
    return memcmp(lhs, rhs, size) == 0;
}

int main()
{
    exo::NodeLiveSampleQueue<8U, 2U> queue;
    const uint8_t first[] = { 1U, 2U, 3U };
    const uint8_t second[] = { 4U, 5U };
    const uint8_t third[] = { 6U };
    exo::NodeLiveSample<8U> out{};

    queue.configure(false, 20U);
    if (queue.offer(1U, first, sizeof(first), 100U) || queue.pop(out)) {
        return 1;
    }

    queue.configure(true, 20U);
    if (!queue.offer(1U, first, sizeof(first), 100U) ||
        queue.offer(1U, second, sizeof(second), 119U) ||
        !queue.offer(2U, second, sizeof(second), 120U)) {
        return 2;
    }
    if (!queue.offer(1U, third, sizeof(third), 140U) || queue.dropped() != 1U) {
        return 3;
    }
    if (!queue.peek(out) || out.sensor_id != 2U ||
        !same_bytes(out.payload, second, sizeof(second)) ||
        !queue.peek(out) || out.sensor_id != 2U) {
        return 4;
    }
    if (!queue.pop(out) || out.sensor_id != 2U || out.payload_len != sizeof(second) ||
        !same_bytes(out.payload, second, sizeof(second))) {
        return 5;
    }
    if (!queue.peek(out) || out.sensor_id != 1U || out.payload_len != sizeof(third) ||
        !same_bytes(out.payload, third, sizeof(third)) ||
        !queue.discard_front() || queue.pop(out)) {
        return 6;
    }
    return 0;
}

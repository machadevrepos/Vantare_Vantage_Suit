#include <cassert>
#include <cstdint>
#include <iostream>

#include <exo/recording/node_live_sample_queue.h>

int main()
{
    exo::NodeLiveSampleQueue<8U, 8U> queue;
    queue.configure(true, 20U);
    // Anything below 40 ms (25 Hz/sensor contract) is clamped to 40.
    assert(queue.normal_interval_ms() == 40U);
    assert(queue.effective_interval_ms() == 40U);

    uint8_t value = 1U;
    assert(queue.offer(1U, &value, 1U, 0U));
    value = 2U;
    assert(queue.offer(2U, &value, 1U, 0U));

    exo::NodeLiveSample<8U> sample{};
    assert(queue.peek(sample));
    assert(sample.sensor_id == 1U);
    assert(queue.discard_front());
    assert(queue.peek(sample));
    assert(sample.sensor_id == 2U);
    assert(queue.discard_front());

    // The per-sensor preview interval gate still decimates within a period.
    value = 3U;
    assert(!queue.offer(1U, &value, 1U, 20U));
    assert(queue.decimated() == 1U);
    assert(queue.offer(1U, &value, 1U, 40U));

    const uint8_t first_queued_value = value;
    for (uint32_t t = 80U; t <= 320U; t += 40U) {
        ++value;
        assert(queue.offer(1U, &value, 1U, t));
    }
    assert(queue.count() == 8U);
    assert(queue.coalesced() == 0U);
    assert(queue.dropped() == 0U);
    assert(queue.peek(sample));
    assert(sample.payload[0] == first_queued_value);

    // A full queue now drops the OLDEST sample and accepts the newest, so the
    // live stream stays current under a slow link instead of latching to a
    // fraction of the contract rate. Loss is still counted.
    ++value;
    const uint8_t newest_value = value;
    assert(queue.offer(1U, &value, 1U, 360U));
    assert(queue.dropped() == 1U);
    assert(!queue.congested());
    assert(queue.effective_interval_ms() == 40U);
    assert(queue.count() == 8U);
    // Oldest (first_queued_value) was dropped; front is now the second sample.
    assert(queue.peek(sample));
    assert(sample.payload[0] == static_cast<uint8_t>(first_queued_value + 1U));

    uint8_t expected = static_cast<uint8_t>(first_queued_value + 1U);
    while (queue.pop(sample)) {
        assert(sample.sensor_id == 1U);
        assert(sample.payload[0] == expected++);
    }
    assert(static_cast<uint8_t>(expected - 1U) == newest_value);
    assert(queue.count() == 0U);

    // Interval gate still applies after draining.
    value = 99U;
    assert(queue.offer(2U, &value, 1U, 5400U));
    assert(!queue.offer(3U, &value, 1U, 5500U));

    std::cout << "node live sample queue tests passed\n";
    return 0;
}

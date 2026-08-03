#include <cassert>
#include <cstdint>
#include <iostream>

#include "NODE_LIVE_SAMPLE_QUEUE.h"

int main()
{
    exo::NodeLiveSampleQueue<8U, 8U> queue;
    queue.configure(true, 20U);
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

    value = 3U;
    assert(!queue.offer(1U, &value, 1U, 20U));
    assert(queue.decimated() == 1U);
    assert(queue.offer(1U, &value, 1U, 40U));

    for (uint32_t t = 80U; t <= 200U; t += 40U) {
        ++value;
        assert(queue.offer(1U, &value, 1U, t));
    }
    assert(queue.coalesced() >= 4U);
    assert(queue.congested());
    assert(queue.effective_interval_ms() == 80U);

    assert(queue.peek(sample));
    assert(sample.sensor_id == 1U);
    assert(sample.payload[0] == value);
    assert(queue.discard_front());

    value = 99U;
    assert(queue.offer(2U, &value, 1U, 5300U));
    assert(!queue.congested());
    assert(queue.effective_interval_ms() == 40U);
    assert(!queue.offer(3U, &value, 1U, 5400U));

    std::cout << "node live sample queue tests passed\n";
    return 0;
}

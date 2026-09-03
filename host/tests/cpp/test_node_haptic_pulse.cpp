#include <cassert>
#include <cstdint>
#include <iostream>

#include <exo/actuator/node_haptic_pulse.h>

/* Design Section 6.4 / Section 12 "Firmware tests":
 * motor stops locally at the duration bound, duplicate and reordered event
 * identifiers do not produce extra pulses, and out-of-range intensity or
 * duration is rejected rather than clamped. */

static exo::HapticPulseRequest make(uint32_t event_id, uint8_t percent, uint16_t duration_ms)
{
    exo::HapticPulseRequest request{};
    request.event_id = event_id;
    request.intensity_percent = percent;
    request.duration_ms = duration_ms;
    return request;
}

int main()
{
    using exo::HapticPulseResult;

    /* The Node owns the stop deadline: no further packet is needed to end it. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(1U, 50U, 250U), 1000U) == HapticPulseResult::Accepted);
        assert(pulse.active());
        assert(pulse.intensity_percent() == 50U);
        assert(!pulse.service(1249U));
        assert(pulse.active());
        assert(pulse.service(1250U));
        assert(!pulse.active());
        assert(pulse.intensity_percent() == 0U);
        assert(pulse.completed_count() == 1U);
        /* Servicing again is idempotent, so a late loop pass cannot re-trigger. */
        assert(!pulse.service(5000U));
    }

    /* A service() call that arrives long after expiry still stops the motor. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(1U, 60U, 100U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.service(90000U));
        assert(!pulse.active());
    }

    /* Bounds are rejected, never clamped. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(1U, 0U, 250U), 0U) == HapticPulseResult::RejectedIntensity);
        assert(pulse.submit(make(2U, 101U, 250U), 0U) == HapticPulseResult::RejectedIntensity);
        assert(pulse.submit(make(3U, 50U, 49U), 0U) == HapticPulseResult::RejectedDuration);
        assert(pulse.submit(make(4U, 50U, 501U), 0U) == HapticPulseResult::RejectedDuration);
        assert(!pulse.active());
        assert(pulse.rejected_count() == 4U);
        /* The boundary values themselves are legal. */
        assert(pulse.submit(make(5U, 1U, 50U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.service(50U));
        assert(pulse.submit(make(6U, 100U, 500U), 100U) == HapticPulseResult::Accepted);
        assert(pulse.service(600U));
    }

    /* Duplicate and reordered identifiers are both suppressed. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(7U, 50U, 250U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.service(250U));
        assert(pulse.submit(make(7U, 50U, 250U), 300U) == HapticPulseResult::RejectedStaleEvent);
        assert(pulse.submit(make(3U, 50U, 250U), 300U) == HapticPulseResult::RejectedStaleEvent);
        assert(!pulse.active());
        assert(pulse.suppressed_count() == 2U);
        assert(pulse.submit(make(8U, 50U, 250U), 300U) == HapticPulseResult::Accepted);
        assert(pulse.active());
    }

    /* cancel() is the fail-silent path used on disarm and disconnect. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(1U, 80U, 400U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.cancel());
        assert(!pulse.active());
        assert(pulse.intensity_percent() == 0U);
        assert(!pulse.cancel());
        assert(pulse.cancelled_count() == 1U);
    }

    /* A new session restarts identifier numbering from scratch. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(9000U, 50U, 250U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.service(250U));
        assert(pulse.submit(make(5U, 50U, 250U), 300U) == HapticPulseResult::RejectedStaleEvent);
        pulse.reset_session();
        assert(!pulse.active());
        assert(pulse.submit(make(5U, 50U, 250U), 300U) == HapticPulseResult::Accepted);
    }

    /* Tick arithmetic survives the HAL_GetTick() wrap. */
    {
        exo::NodeHapticPulse pulse;
        const uint32_t near_wrap = 0xFFFFFFF0UL; /* +250 crosses the 2^32 boundary */
        assert(pulse.submit(make(1U, 50U, 250U), near_wrap) == HapticPulseResult::Accepted);
        assert(!pulse.service(static_cast<uint32_t>(near_wrap + 249U)));
        assert(pulse.service(static_cast<uint32_t>(near_wrap + 250U)));
        assert(!pulse.active());
    }

    /* Identifier comparison also survives the counter wrap. */
    {
        exo::NodeHapticPulse pulse;
        assert(pulse.submit(make(0xFFFFFFFEUL, 50U, 250U), 0U) == HapticPulseResult::Accepted);
        assert(pulse.service(250U));
        assert(pulse.submit(make(2U, 50U, 250U), 300U) == HapticPulseResult::Accepted);
    }

    std::cout << "test_node_haptic_pulse passed" << std::endl;
    return 0;
}

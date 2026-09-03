#ifndef NODE_HAPTIC_PULSE_H_
#define NODE_HAPTIC_PULSE_H_

#include <stdint.h>

namespace exo {

/**
 * Node-owned bounded haptic pulse (design Section 6.4).
 *
 * The Node owns the stop deadline so a lost follow-up packet cannot leave a
 * motor running against a wearer's arm. The browser only asks for a pulse; it
 * never owns the off timer.
 *
 * Pure logic, no HAL: the caller supplies a millisecond tick and drives the
 * PWM from intensity_percent(). All tick arithmetic is wrap-safe.
 */

enum class HapticPulseResult : uint8_t {
    Accepted = 0U,
    RejectedIntensity = 1U,
    RejectedDuration = 2U,
    RejectedStaleEvent = 3U,
};

struct HapticPulseRequest {
    uint32_t event_id;
    uint8_t intensity_percent;
    uint16_t duration_ms;
};

class NodeHapticPulse {
public:
    static constexpr uint8_t kMinIntensityPercent = 1U;
    static constexpr uint8_t kMaxIntensityPercent = 100U;
    static constexpr uint16_t kMinDurationMs = 50U;
    static constexpr uint16_t kMaxDurationMs = 500U;

    /**
     * Validate and start a pulse. Bounds are rejected rather than clamped so a
     * malformed request never becomes a long pulse. An event identifier at or
     * below the highest already executed is ignored, which suppresses both an
     * exact retransmission and a reordered stale command.
     */
    HapticPulseResult submit(const HapticPulseRequest &request, uint32_t now_ms)
    {
        if (request.intensity_percent < kMinIntensityPercent ||
            request.intensity_percent > kMaxIntensityPercent) {
            ++rejected_count_;
            return HapticPulseResult::RejectedIntensity;
        }
        if (request.duration_ms < kMinDurationMs || request.duration_ms > kMaxDurationMs) {
            ++rejected_count_;
            return HapticPulseResult::RejectedDuration;
        }
        if (has_executed_ && !event_is_newer(request.event_id)) {
            ++suppressed_count_;
            return HapticPulseResult::RejectedStaleEvent;
        }
        has_executed_ = true;
        last_event_id_ = request.event_id;
        active_ = true;
        intensity_percent_ = request.intensity_percent;
        start_ms_ = now_ms;
        duration_ms_ = request.duration_ms;
        ++accepted_count_;
        return HapticPulseResult::Accepted;
    }

    /**
     * Drive from the main loop. Returns true exactly once, on the tick the
     * pulse expires, so the caller can set the motor to zero. Calling it late
     * still stops the motor; calling it often costs one comparison.
     */
    bool service(uint32_t now_ms)
    {
        if (!active_) {
            return false;
        }
        if (static_cast<uint32_t>(now_ms - start_ms_) < duration_ms_) {
            return false;
        }
        active_ = false;
        intensity_percent_ = 0U;
        ++completed_count_;
        return true;
    }

    /** Force the motor off (disarm, disconnect, session stop, fault). */
    bool cancel()
    {
        if (!active_) {
            return false;
        }
        active_ = false;
        intensity_percent_ = 0U;
        ++cancelled_count_;
        return true;
    }

    /** Clears the identifier history; a new live session restarts numbering. */
    void reset_session()
    {
        cancel();
        has_executed_ = false;
        last_event_id_ = 0U;
    }

    bool active() const { return active_; }
    uint8_t intensity_percent() const { return intensity_percent_; }
    uint32_t last_event_id() const { return last_event_id_; }
    uint32_t accepted_count() const { return accepted_count_; }
    uint32_t completed_count() const { return completed_count_; }
    uint32_t rejected_count() const { return rejected_count_; }
    uint32_t suppressed_count() const { return suppressed_count_; }
    uint32_t cancelled_count() const { return cancelled_count_; }

private:
    /* Wrap-safe "newer than last executed" over a monotonically increasing
     * session counter: treat the nearer half of the identifier space as the
     * future so a session that runs past 2^31 events keeps working. */
    bool event_is_newer(uint32_t event_id) const
    {
        return static_cast<int32_t>(event_id - last_event_id_) > 0;
    }

    bool active_ { false };
    bool has_executed_ { false };
    uint8_t intensity_percent_ { 0U };
    uint16_t duration_ms_ { 0U };
    uint32_t start_ms_ { 0U };
    uint32_t last_event_id_ { 0U };
    uint32_t accepted_count_ { 0U };
    uint32_t completed_count_ { 0U };
    uint32_t rejected_count_ { 0U };
    uint32_t suppressed_count_ { 0U };
    uint32_t cancelled_count_ { 0U };
};

} // namespace exo

#endif

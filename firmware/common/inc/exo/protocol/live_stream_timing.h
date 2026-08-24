#ifndef LIVE_STREAM_TIMING_H_
#define LIVE_STREAM_TIMING_H_

#include <stdint.h>

namespace exo {

class LiveStreamGate {
public:
    constexpr bool accept(bool fresh, uint32_t now_ms, uint32_t interval_ms)
    {
        if (!fresh) {
            return false;
        }
        if (!initialized_) {
            initialized_ = true;
            last_accepted_ms_ = now_ms;
            return true;
        }
        if (static_cast<uint32_t>(now_ms - last_accepted_ms_) < interval_ms) {
            return false;
        }
        last_accepted_ms_ = now_ms;
        return true;
    }

private:
    bool initialized_ = false;
    uint32_t last_accepted_ms_ = 0U;
};

} // namespace exo

#endif // LIVE_STREAM_TIMING_H_

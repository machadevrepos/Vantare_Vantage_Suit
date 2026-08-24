#ifndef MASTER_SESSION_TIMESTAMP_LEDGER_H_
#define MASTER_SESSION_TIMESTAMP_LEDGER_H_

#include <stdint.h>

namespace exo {

class MasterSessionTimestampLedger {
public:
    static constexpr uint32_t kMaxIcmSamples = 2400U;

    void reset(uint32_t session_id)
    {
        session_id_ = session_id;
        icm_count_ = 0U;
        overflowed_ = false;
    }

    bool append_icm(uint32_t session_time_us)
    {
        if (icm_count_ >= kMaxIcmSamples) {
            overflowed_ = true;
            return false;
        }

        icm_times_us_[icm_count_++] = session_time_us;
        return true;
    }

    bool icm_time(uint32_t index, uint32_t &out) const
    {
        if (index >= icm_count_) {
            return false;
        }

        out = icm_times_us_[index];
        return true;
    }

    uint32_t session_id() const
    {
        return session_id_;
    }

    uint32_t icm_count() const
    {
        return icm_count_;
    }

    bool overflowed() const
    {
        return overflowed_;
    }

private:
    uint32_t icm_times_us_[kMaxIcmSamples]{};
    uint32_t session_id_ = 0U;
    uint32_t icm_count_ = 0U;
    bool overflowed_ = false;
};

} // namespace exo

#endif /* MASTER_SESSION_TIMESTAMP_LEDGER_H_ */

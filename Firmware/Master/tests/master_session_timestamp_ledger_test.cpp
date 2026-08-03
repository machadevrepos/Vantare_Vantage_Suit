#include <stdint.h>

#include "MASTER_SESSION_TIMESTAMP_LEDGER.h"

static_assert(exo::MasterSessionTimestampLedger::kMaxIcmSamples == 2400U,
        "Master ICM timestamp ledger capacity must remain 2400 samples");
static_assert(exo::MasterSessionTimestampLedger::kMaxIcmSamples * sizeof(uint32_t) == 9600U,
        "Master ICM timestamp ledger storage must remain 9600 bytes");

int main()
{
    exo::MasterSessionTimestampLedger ledger;
    ledger.reset(42U);
    if (!ledger.append_icm(10000U) || !ledger.append_icm(15500U)) {
        return 1;
    }

    uint32_t time_us = 0U;
    if (!ledger.icm_time(1U, time_us) || time_us != 15500U) {
        return 2;
    }
    if (ledger.session_id() != 42U || ledger.icm_count() != 2U) {
        return 3;
    }

    for (uint32_t index = 2U;
            index < exo::MasterSessionTimestampLedger::kMaxIcmSamples;
            ++index) {
        if (!ledger.append_icm(20000U + index)) {
            return 4;
        }
    }
    if (ledger.icm_count() != exo::MasterSessionTimestampLedger::kMaxIcmSamples ||
            ledger.append_icm(999999U) || !ledger.overflowed()) {
        return 5;
    }
    if (!ledger.icm_time(0U, time_us) || time_us != 10000U ||
            !ledger.icm_time(1U, time_us) || time_us != 15500U ||
            !ledger.icm_time(exo::MasterSessionTimestampLedger::kMaxIcmSamples - 1U, time_us) ||
            time_us != (20000U + exo::MasterSessionTimestampLedger::kMaxIcmSamples - 1U)) {
        return 6;
    }

    ledger.reset(43U);
    if (ledger.session_id() != 43U || ledger.icm_count() != 0U || ledger.overflowed() ||
            ledger.icm_time(0U, time_us)) {
        return 7;
    }
    if (!ledger.append_icm(25000U) || !ledger.icm_time(0U, time_us) || time_us != 25000U) {
        return 8;
    }

    return 0;
}

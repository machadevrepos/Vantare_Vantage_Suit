#ifndef ACQUISITION_DIAGNOSTICS_H_
#define ACQUISITION_DIAGNOSTICS_H_

/* Phase 1 acquisition instrumentation.
 *
 * The Master captures BNO85 near 31 Hz and ICM45686 near 66 Hz while the Node
 * reaches 200 Hz with the same recorder and sample format. Both Master streams
 * stall for roughly the same ~256 ms, which points at a shared blocking step in
 * the superloop rather than at either sensor driver. Rather than guess which
 * step, this header accumulates plain counters in the hot path and prints one
 * compact summary once the capture has already finished.
 *
 * Hot-path cost is a monotonic microsecond read plus integer arithmetic. There
 * is no per-sample formatting, no printf and no storage access while capturing.
 *
 * Everything below the clock helper is constexpr and free of target headers so
 * the accounting can be exercised at compile time by the host test.
 */

#include <stdint.h>

/* Master accounting on/off. The counters are cheap enough to leave on for the
 * hardware runs; a build with this at 0 must be byte-for-byte the old behavior. */
#ifndef EXO_ACQ_DIAG_ENABLE
#define EXO_ACQ_DIAG_ENABLE 1
#endif

/* ---- Experiment switches. All default off: none of these may change a
 * production build. Each one is meant to be enabled on its own for a single
 * 15 s hardware capture so one suspect can be removed at a time. ---- */

/* B: skip recorder appends while capturing. Acquisition counters keep running,
 * so this separates "sensor cannot produce" from "SD write blocks the loop". */
#ifndef EXO_ACQ_DIAG_SUPPRESS_SD
#define EXO_ACQ_DIAG_SUPPRESS_SD 0
#endif

/* C: drop nonessential BLE notifications and SWO chatter while capturing.
 * Record-control traffic is deliberately left alone. */
#ifndef EXO_ACQ_DIAG_QUIET_COMMS
#define EXO_ACQ_DIAG_QUIET_COMMS 0
#endif

/* D1: Master reads ICM only. */
#ifndef EXO_ACQ_DIAG_ICM_ONLY
#define EXO_ACQ_DIAG_ICM_ONLY 0
#endif

/* D2 (paired with the run matrix): Master reads BNO only. */
#ifndef EXO_ACQ_DIAG_BNO_ONLY
#define EXO_ACQ_DIAG_BNO_ONLY 0
#endif

/* D2: BNO publishes game rotation vector only, so the auxiliary linear
 * acceleration, gravity and gyro reports stop competing for SHTP bandwidth. */
#ifndef EXO_ACQ_DIAG_BNO_RV_ONLY
#define EXO_ACQ_DIAG_BNO_RV_ONLY 0
#endif

/* The quiet-comms experiment keys off the live capturing flag, which only
 * exists when the accounting is compiled in. */
#if EXO_ACQ_DIAG_QUIET_COMMS && !EXO_ACQ_DIAG_ENABLE
#error "EXO_ACQ_DIAG_QUIET_COMMS requires EXO_ACQ_DIAG_ENABLE"
#endif

#if EXO_ACQ_DIAG_ICM_ONLY && EXO_ACQ_DIAG_BNO_ONLY
#error "EXO_ACQ_DIAG_ICM_ONLY and EXO_ACQ_DIAG_BNO_ONLY are mutually exclusive"
#endif

namespace exo {
namespace diag {

/* ------------------------------------------------------------------ */
/* Duration accounting                                                 */
/* ------------------------------------------------------------------ */

/* Latency buckets are cumulative: a 30 ms sample lands in over_5ms, over_10ms
 * and over_20ms. Cumulative buckets read directly as "how often did this step
 * blow the 5 ms ICM deadline", which is the question the run matrix asks. */
struct LatencyStat {
    uint32_t count = 0U;
    uint32_t max_us = 0U;
    uint64_t total_us = 0U;
    uint32_t over_5ms = 0U;
    uint32_t over_10ms = 0U;
    uint32_t over_20ms = 0U;
    uint32_t over_100ms = 0U;

    constexpr void reset() { *this = LatencyStat{}; }

    constexpr void note(uint32_t duration_us) {
        ++count;
        total_us += duration_us;
        if (duration_us > max_us) {
            max_us = duration_us;
        }
        if (duration_us > 5000U) {
            ++over_5ms;
        }
        if (duration_us > 10000U) {
            ++over_10ms;
        }
        if (duration_us > 20000U) {
            ++over_20ms;
        }
        if (duration_us > 100000U) {
            ++over_100ms;
        }
    }

    constexpr uint32_t mean_us() const {
        return (count == 0U) ? 0U : static_cast<uint32_t>(total_us / count);
    }
};

/* Spacing between successive captured samples. The first sample only seeds the
 * reference; a gap needs two samples to exist. */
struct GapStat {
    uint32_t count = 0U;
    uint32_t max_gap_us = 0U;
    uint64_t last_us = 0U;
    bool have_last = false;

    constexpr void reset() { *this = GapStat{}; }

    constexpr void note(uint64_t now_us) {
        ++count;
        if (have_last && now_us > last_us) {
            const uint64_t gap = now_us - last_us;
            if (gap > max_gap_us) {
                max_gap_us = static_cast<uint32_t>(gap);
            }
        }
        last_us = now_us;
        have_last = true;
    }
};

/* ------------------------------------------------------------------ */
/* BNO report identifiers                                              */
/* ------------------------------------------------------------------ */

/* Mirrors the four reports Bno85Stm32::begin() enables, kept local so this
 * header stays free of the sh2 stack and remains host-compilable. */
enum BnoReportSlot : uint8_t {
    kBnoSlotRotation = 0U,
    kBnoSlotLinearAccel = 1U,
    kBnoSlotGravity = 2U,
    kBnoSlotGyro = 3U,
    kBnoSlotOther = 4U,
    kBnoSlotCount = 5U
};

constexpr BnoReportSlot bno_slot_for_report(uint8_t report_id) {
    switch (report_id) {
    case 0x08U:
        return kBnoSlotRotation;  // SH2_GAME_ROTATION_VECTOR
    case 0x04U:
        return kBnoSlotLinearAccel;  // SH2_LINEAR_ACCELERATION
    case 0x06U:
        return kBnoSlotGravity;  // SH2_GRAVITY
    case 0x02U:
        return kBnoSlotGyro;  // SH2_GYROSCOPE_CALIBRATED
    default:
        return kBnoSlotOther;
    }
}

/* ------------------------------------------------------------------ */
/* Aggregate                                                           */
/* ------------------------------------------------------------------ */

struct AcquisitionDiagnostics {
    /* 1. Main superloop */
    LatencyStat loop;

    /* 2. BNO */
    uint32_t bno_service_calls = 0U;
    uint32_t bno_take_latest_ok = 0U;
    uint32_t bno_sensor_events = 0U;
    uint32_t bno_i2c_errors = 0U;
    uint32_t bno_reports[kBnoSlotCount] = {0U, 0U, 0U, 0U, 0U};
    LatencyStat bno_service;
    GapStat bno_gap;

    /* 3. ICM */
    uint32_t icm_attempts = 0U;
    uint32_t icm_ok = 0U;
    uint32_t icm_fail = 0U;
    LatencyStat icm_read;
    GapStat icm_gap;

    /* 4. Recorder / SD */
    uint32_t sd_bno_append_ok = 0U;
    uint32_t sd_bno_append_fail = 0U;
    uint32_t sd_icm_append_ok = 0U;
    uint32_t sd_icm_append_fail = 0U;
    uint32_t sd_flushes = 0U;
    LatencyStat sd_write;

    /* 5. Communications */
    LatencyStat comms_ble;
    LatencyStat comms_central;
    LatencyStat comms_leaf;

    /* Session bookkeeping */
    bool capturing = false;
    bool summary_pending = false;
    uint32_t summaries_emitted = 0U;
    uint64_t session_start_us = 0U;
    uint64_t session_end_us = 0U;

    /* Every counter is cleared here, so a second session never inherits the
     * first session's maxima. summaries_emitted deliberately survives: the test
     * uses it to prove one summary per completed session. */
    constexpr void begin_session(uint64_t now_us) {
        const uint32_t emitted = summaries_emitted;
        *this = AcquisitionDiagnostics{};
        summaries_emitted = emitted;
        capturing = true;
        session_start_us = now_us;
    }

    constexpr void end_session(uint64_t now_us) {
        if (!capturing) {
            return;
        }
        capturing = false;
        session_end_us = now_us;
        summary_pending = true;
    }

    constexpr uint32_t session_duration_ms() const {
        return (session_end_us > session_start_us)
                   ? static_cast<uint32_t>((session_end_us - session_start_us) / 1000ULL)
                   : 0U;
    }

    /* Claims the pending summary. Returns true exactly once per completed
     * session, which is what keeps the print out of the capture loop. */
    constexpr bool claim_summary() {
        if (!summary_pending) {
            return false;
        }
        summary_pending = false;
        ++summaries_emitted;
        return true;
    }

    /* Whole samples per second, and the hundredths, without floating point. */
    constexpr uint32_t rate_centihz(uint32_t samples) const {
        const uint32_t ms = session_duration_ms();
        if (ms == 0U) {
            return 0U;
        }
        return static_cast<uint32_t>((static_cast<uint64_t>(samples) * 100000ULL) / ms);
    }

    constexpr void note_bno_report(uint8_t report_id) {
        ++bno_reports[bno_slot_for_report(report_id)];
    }
};

}  // namespace diag
}  // namespace exo

/* ------------------------------------------------------------------ */
/* Target-side monotonic microsecond clock                             */
/* ------------------------------------------------------------------ */

/* Host tests only need the accounting above, so the DWT helper is skipped when
 * the CMSIS core header is not in the include path. */
#if EXO_ACQ_DIAG_ENABLE && !defined(EXO_ACQ_DIAG_HOST_TEST)

namespace exo {
namespace diag {

/* DWT CYCCNT wraps every ~67 s at 64 MHz, which is shorter than a session, so
 * the wrap is folded into a 64-bit accumulator. Safe because the superloop
 * samples this far more often than once per wrap period.
 *
 * HAL_GetTick() is deliberately not used: at 1 ms resolution it cannot resolve
 * the 5 ms ICM budget this instrumentation exists to measure. */
class MicroClock {
public:
    static void begin() {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0U;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        cycles_per_us_ = SystemCoreClock / 1000000U;
        if (cycles_per_us_ == 0U) {
            cycles_per_us_ = 1U;
        }
        last_cycles_ = DWT->CYCCNT;
        accumulated_us_ = 0U;
        started_ = true;
    }

    static bool started() { return started_; }

    static uint64_t now_us() {
        if (!started_) {
            return 0U;
        }
        const uint32_t now_cycles = DWT->CYCCNT;
        const uint32_t delta_cycles = now_cycles - last_cycles_;  // wrap-safe
        last_cycles_ = now_cycles;
        accumulated_us_ += delta_cycles / cycles_per_us_;
        /* Cycles that did not add up to a whole microsecond are carried so the
         * clock does not lose time across many short reads. */
        carry_cycles_ += delta_cycles % cycles_per_us_;
        if (carry_cycles_ >= cycles_per_us_) {
            accumulated_us_ += carry_cycles_ / cycles_per_us_;
            carry_cycles_ %= cycles_per_us_;
        }
        return accumulated_us_;
    }

private:
    inline static bool started_ = false;
    inline static uint32_t cycles_per_us_ = 1U;
    inline static uint32_t last_cycles_ = 0U;
    inline static uint32_t carry_cycles_ = 0U;
    inline static uint64_t accumulated_us_ = 0U;
};

/* Scoped stopwatch feeding one LatencyStat. */
class ScopedLatency {
public:
    explicit ScopedLatency(LatencyStat &stat) : stat_(stat), start_us_(MicroClock::now_us()) {}
    ~ScopedLatency() { stat_.note(static_cast<uint32_t>(MicroClock::now_us() - start_us_)); }

    ScopedLatency(const ScopedLatency &) = delete;
    ScopedLatency &operator=(const ScopedLatency &) = delete;

private:
    LatencyStat &stat_;
    uint64_t start_us_;
};

}  // namespace diag
}  // namespace exo

#define EXO_ACQ_DIAG_CAT2(a, b) a##b
#define EXO_ACQ_DIAG_CAT(a, b) EXO_ACQ_DIAG_CAT2(a, b)
#define EXO_ACQ_DIAG_NOW_US() ::exo::diag::MicroClock::now_us()
#define EXO_ACQ_DIAG_SCOPE(stat) \
    ::exo::diag::ScopedLatency EXO_ACQ_DIAG_CAT(exo_diag_scope_, __LINE__)((stat))

#else /* diagnostics compiled out */

#define EXO_ACQ_DIAG_NOW_US() (0ULL)
#define EXO_ACQ_DIAG_SCOPE(stat) ((void)0)

#endif

#endif /* ACQUISITION_DIAGNOSTICS_H_ */

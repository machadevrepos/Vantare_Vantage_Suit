/* Behavioral test for the Phase 1 acquisition counters.
 *
 * There is no host compiler in this tree, so the accounting is written as
 * constexpr and exercised with static_assert: the ARM `g++ -fsyntax-only`
 * check actually evaluates every case below at compile time. A regression in
 * bucket accounting, maximum tracking, session reset or summary claiming is a
 * compile error, not a silent pass.
 *
 * Build:
 *   arm-none-eabi-g++ -std=gnu++17 -fsyntax-only \
 *     -DEXO_ACQ_DIAG_HOST_TEST=1 -I Firmware/LIBRARY/CUSTOM \
 *     Firmware/Master/tests/acquisition_diagnostics_test.cpp
 */

#include <stdint.h>

#define EXO_ACQ_DIAG_HOST_TEST 1

#include "ACQUISITION_DIAGNOSTICS.h"

namespace {

using exo::diag::AcquisitionDiagnostics;
using exo::diag::GapStat;
using exo::diag::LatencyStat;

/* ---------------- latency bucket accounting ---------------- */

constexpr LatencyStat build_latency() {
    LatencyStat stat{};
    stat.note(1000U);    // under every threshold
    stat.note(5000U);    // exactly 5 ms is not "over 5 ms"
    stat.note(5001U);    // over 5 ms only
    stat.note(12000U);   // over 5 and 10
    stat.note(25000U);   // over 5, 10 and 20
    stat.note(256000U);  // the observed Master stall: every bucket
    return stat;
}

constexpr LatencyStat kLatency = build_latency();

static_assert(kLatency.count == 6U, "every sample must be counted");
static_assert(kLatency.max_us == 256000U, "maximum duration must track the worst sample");
static_assert(kLatency.total_us == 304001U, "total must accumulate every sample");

/* Buckets are cumulative, so one 256 ms stall lands in all four. */
static_assert(kLatency.over_5ms == 4U, "over_5ms must count 5001, 12000, 25000, 256000");
static_assert(kLatency.over_10ms == 3U, "over_10ms must count 12000, 25000, 256000");
static_assert(kLatency.over_20ms == 2U, "over_20ms must count 25000, 256000");
static_assert(kLatency.over_100ms == 1U, "over_100ms must count 256000 only");

/* Boundary values belong to the lower bucket. */
constexpr LatencyStat build_boundaries() {
    LatencyStat stat{};
    stat.note(5000U);
    stat.note(10000U);
    stat.note(20000U);
    stat.note(100000U);
    return stat;
}

constexpr LatencyStat kBoundaries = build_boundaries();
static_assert(kBoundaries.over_5ms == 3U, "10/20/100 ms exceed 5 ms; 5 ms itself does not");
static_assert(kBoundaries.over_10ms == 2U, "20/100 ms exceed 10 ms; 10 ms itself does not");
static_assert(kBoundaries.over_20ms == 1U, "100 ms exceeds 20 ms; 20 ms itself does not");
static_assert(kBoundaries.over_100ms == 0U, "exactly 100 ms is not over 100 ms");

static_assert(kBoundaries.mean_us() == 33750U, "mean must be the integer average");
static_assert(LatencyStat{}.mean_us() == 0U, "an empty stat must not divide by zero");

constexpr LatencyStat build_reset() {
    LatencyStat stat = build_latency();
    stat.reset();
    return stat;
}

constexpr LatencyStat kReset = build_reset();
static_assert(kReset.count == 0U && kReset.max_us == 0U && kReset.total_us == 0U,
              "reset must clear counts and maxima");
static_assert(kReset.over_5ms == 0U && kReset.over_100ms == 0U, "reset must clear buckets");

/* ---------------- sample gap accounting ---------------- */

constexpr GapStat build_gap() {
    GapStat stat{};
    stat.note(1000U);     // first sample only seeds the reference
    stat.note(6000U);     // 5 ms
    stat.note(267000U);   // 261 ms, the observed Master BNO stall
    stat.note(272000U);   // 5 ms
    return stat;
}

constexpr GapStat kGap = build_gap();
static_assert(kGap.count == 4U, "every captured sample must be counted");
static_assert(kGap.max_gap_us == 261000U, "the worst gap must survive later short gaps");
static_assert(kGap.have_last && kGap.last_us == 272000U, "the reference must follow the last sample");

constexpr GapStat build_single_gap() {
    GapStat stat{};
    stat.note(50000U);
    return stat;
}
static_assert(build_single_gap().max_gap_us == 0U,
              "one sample cannot produce a gap");

/* ---------------- BNO report classification ---------------- */

static_assert(exo::diag::bno_slot_for_report(0x08U) == exo::diag::kBnoSlotRotation, "0x08 is GRV");
static_assert(exo::diag::bno_slot_for_report(0x04U) == exo::diag::kBnoSlotLinearAccel, "0x04 is linear accel");
static_assert(exo::diag::bno_slot_for_report(0x06U) == exo::diag::kBnoSlotGravity, "0x06 is gravity");
static_assert(exo::diag::bno_slot_for_report(0x02U) == exo::diag::kBnoSlotGyro, "0x02 is calibrated gyro");
static_assert(exo::diag::bno_slot_for_report(0x7FU) == exo::diag::kBnoSlotOther, "unknown ids fall through");

/* ---------------- session lifecycle ---------------- */

/* Populate a session the way a capture would, then start a second one. */
constexpr AcquisitionDiagnostics build_session_one() {
    AcquisitionDiagnostics d{};
    d.begin_session(1'000'000U);
    d.loop.note(256000U);
    d.bno_service_calls = 400U;
    d.bno_take_latest_ok = 408U;
    d.icm_ok = 877U;
    d.sd_icm_append_ok = 877U;
    d.note_bno_report(0x08U);
    d.note_bno_report(0x08U);
    d.note_bno_report(0x02U);
    d.bno_gap.note(1'100'000U);
    d.bno_gap.note(1'361'000U);
    d.end_session(14'000'000U);
    return d;
}

constexpr AcquisitionDiagnostics kSession1 = build_session_one();

static_assert(kSession1.capturing == false, "end_session must clear the capturing flag");
static_assert(kSession1.summary_pending, "end_session must queue exactly one summary");
static_assert(kSession1.session_duration_ms() == 13000U, "duration must be end minus start");
static_assert(kSession1.bno_reports[exo::diag::kBnoSlotRotation] == 2U, "GRV reports must accumulate");
static_assert(kSession1.bno_reports[exo::diag::kBnoSlotGyro] == 1U, "gyro reports must accumulate");
static_assert(kSession1.bno_gap.max_gap_us == 261000U, "session gap tracking must work end to end");

/* 408 samples over 13 s is 31.38 Hz, reported in hundredths of a hertz. */
static_assert(kSession1.rate_centihz(408U) == 3138U, "rate must match the measured Master BNO rate");
static_assert(kSession1.rate_centihz(877U) == 6746U, "rate must match the measured Master ICM rate");
static_assert(AcquisitionDiagnostics{}.rate_centihz(100U) == 0U,
              "a zero-length session must not divide by zero");

/* Counters must reset at session start; the emitted-summary tally must not. */
constexpr AcquisitionDiagnostics build_session_two() {
    AcquisitionDiagnostics d = build_session_one();
    (void)d.claim_summary();
    d.begin_session(20'000'000U);
    return d;
}

constexpr AcquisitionDiagnostics kSession2 = build_session_two();

static_assert(kSession2.capturing, "begin_session must arm capturing");
static_assert(kSession2.loop.count == 0U, "loop counters must reset at session start");
static_assert(kSession2.loop.max_us == 0U, "loop maxima must reset at session start");
static_assert(kSession2.bno_take_latest_ok == 0U, "BNO counters must reset at session start");
static_assert(kSession2.icm_ok == 0U, "ICM counters must reset at session start");
static_assert(kSession2.sd_icm_append_ok == 0U, "recorder counters must reset at session start");
static_assert(kSession2.bno_reports[exo::diag::kBnoSlotRotation] == 0U,
              "per-report counters must reset at session start");
static_assert(kSession2.bno_gap.max_gap_us == 0U, "gap maxima must reset at session start");
static_assert(kSession2.bno_gap.have_last == false, "the gap reference must reset at session start");
static_assert(kSession2.summary_pending == false, "a fresh session must not start with a pending summary");
static_assert(kSession2.summaries_emitted == 1U,
              "the emitted-summary tally must survive a reset so one-per-session is provable");

/* ---------------- exactly one summary per completed session ---------------- */

constexpr uint32_t claim_attempts(uint32_t attempts) {
    AcquisitionDiagnostics d = build_session_one();
    uint32_t claimed = 0U;
    for (uint32_t i = 0U; i < attempts; ++i) {
        if (d.claim_summary()) {
            ++claimed;
        }
    }
    return claimed;
}

static_assert(claim_attempts(1U) == 1U, "the first drain must emit the summary");
static_assert(claim_attempts(50U) == 1U,
              "repeated superloop drains must not re-emit a session summary");

constexpr uint32_t claim_without_session() {
    AcquisitionDiagnostics d{};
    uint32_t claimed = 0U;
    for (uint32_t i = 0U; i < 10U; ++i) {
        if (d.claim_summary()) {
            ++claimed;
        }
    }
    return claimed;
}
static_assert(claim_without_session() == 0U, "no session means no summary");

/* Two full sessions must produce two summaries, never one and never three. */
constexpr uint32_t two_session_summaries() {
    AcquisitionDiagnostics d{};
    uint32_t claimed = 0U;
    for (uint32_t session = 0U; session < 2U; ++session) {
        d.begin_session(session * 1'000'000U);
        if (d.claim_summary()) {
            ++claimed;  // must not fire: the session is still capturing
        }
        d.end_session((session * 1'000'000U) + 15'000'000U);
        if (d.claim_summary()) {
            ++claimed;
        }
        if (d.claim_summary()) {
            ++claimed;
        }
    }
    return claimed;
}
static_assert(two_session_summaries() == 2U, "one summary per completed session");

/* end_session on a session that never began must stay silent. */
constexpr bool end_without_begin() {
    AcquisitionDiagnostics d{};
    d.end_session(5'000'000U);
    return d.summary_pending;
}
static_assert(end_without_begin() == false, "end_session without begin_session must not queue a summary");

}  // namespace

/* -fsyntax-only never links, but a definition keeps the file a valid TU. */
int main() { return 0; }

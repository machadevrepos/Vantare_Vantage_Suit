#include <stdint.h>

#include "../../LIBRARY/CUSTOM/LIVE_STREAM_TIMING.h"

constexpr bool first_fresh_sample_is_immediate()
{
    exo::LiveStreamGate gate;
    return gate.accept(true, 1000U, 20U);
}

constexpr bool stale_and_early_samples_are_rejected()
{
    exo::LiveStreamGate gate;
    return gate.accept(true, 1000U, 20U) &&
           !gate.accept(false, 1020U, 20U) &&
           !gate.accept(true, 1019U, 20U) &&
           gate.accept(true, 1020U, 20U);
}

constexpr bool sensor_gates_are_independent()
{
    exo::LiveStreamGate bno;
    exo::LiveStreamGate icm;
    return bno.accept(true, 500U, 20U) &&
           icm.accept(true, 505U, 20U) &&
           !bno.accept(true, 519U, 20U) &&
           icm.accept(true, 525U, 20U);
}

constexpr bool tick_rollover_is_safe()
{
    exo::LiveStreamGate gate;
    return gate.accept(true, 0xFFFFFFF8U, 20U) &&
           !gate.accept(true, 5U, 20U) &&
           gate.accept(true, 12U, 20U);
}

static_assert(first_fresh_sample_is_immediate(), "First fresh sample must stream immediately");
static_assert(stale_and_early_samples_are_rejected(), "Only fresh due samples may stream");
static_assert(sensor_gates_are_independent(), "BNO and ICM timing must be independent");
static_assert(tick_rollover_is_safe(), "HAL tick rollover must not break scheduling");

int main()
{
    return 0;
}

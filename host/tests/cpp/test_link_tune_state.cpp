#include <cstdint>
#include <iostream>

#include <exo/ble/link_tune_state.h>
#include <exo/protocol/record_transfer_tuning.h>

namespace {

int failures = 0;

#define EXPECT_TRUE(expr) do { \
  if (!(expr)) { \
    std::cerr << __FILE__ << ':' << __LINE__ << ": expected " #expr "\n"; \
    ++failures; \
  } \
} while (0)

using Model = exo::LinkTuneState;
using State = Model::State;
using Procedure = Model::Procedure;
using TuningWire = exo::RecordTransferTuningWire;

void complete_fast_tune(Model &model, uint8_t link, uint16_t handle,
                        uint32_t generation, uint32_t &now)
{
  Model::Request request = model.issue_next(now);
  EXPECT_TRUE(request.valid());
  EXPECT_TRUE(request.link == link);
  EXPECT_TRUE(request.handle == handle);
  EXPECT_TRUE(request.generation == generation);
  EXPECT_TRUE(request.procedure == Procedure::Dle);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  EXPECT_TRUE(model.on_dle_complete(handle, generation, 251U, 251U, now));

  request = model.issue_next(now);
  EXPECT_TRUE(request.valid());
  EXPECT_TRUE(request.procedure == Procedure::Phy);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  EXPECT_TRUE(model.on_phy_complete(handle, generation, Model::kStatusSuccess, 2U, 2U, now));

  EXPECT_TRUE(model.telemetry(link).state == State::Ready);
}

void test_serialized_four_link_arbitration()
{
  Model model;
  uint32_t now = 100U;
  for (uint8_t link = 0U; link < 4U; ++link) {
    EXPECT_TRUE(model.connect(link, static_cast<uint16_t>(0x0040U + link), now));
  }
  now += Model::kCommissioningStartDelayMs;

  for (uint8_t link = 0U; link < 4U; ++link) {
    const uint16_t handle = static_cast<uint16_t>(0x0040U + link);
    const uint32_t generation = model.telemetry(link).generation;
    complete_fast_tune(model, link, handle, generation, now);
    EXPECT_TRUE(model.telemetry(link).state == State::Ready);
  }
  EXPECT_TRUE(!model.issue_next(now).valid());
}

void test_transient_error_retries_without_releasing_the_global_arbiter()
{
  Model model;
  uint32_t now = 10U;
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  now += Model::kCommissioningStartDelayMs;
  const Model::Request first = model.issue_next(now);
  EXPECT_TRUE(first.procedure == Procedure::Dle);
  EXPECT_TRUE(model.on_request_status(first, Model::kStatusCommandDisallowed, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::NeedDle);
  EXPECT_TRUE(model.telemetry(0U).retries == 1U);
  EXPECT_TRUE(!model.issue_next(now).valid());
  now += Model::kRetryDelayMs;
  const Model::Request retry = model.issue_next(now);
  EXPECT_TRUE(retry.valid());
  EXPECT_TRUE(retry.procedure == Procedure::Dle);
  EXPECT_TRUE(retry.generation == first.generation);
}

void test_transient_failures_are_bounded_and_degrade()
{
  Model model;
  uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
  for (uint8_t attempt = 0U; attempt < Model::kMaxTransientAttempts; ++attempt) {
    const Model::Request request = model.issue_next(now);
    EXPECT_TRUE(request.valid());
    EXPECT_TRUE(model.on_request_status(request, Model::kStatusCommandDisallowed, now));
    now += Model::kRetryDelayMs;
  }
  EXPECT_TRUE(model.telemetry(0U).state == State::Degraded);
  EXPECT_TRUE(model.collection_allowed(0U));
  EXPECT_TRUE(model.telemetry(0U).retries == Model::kMaxTransientAttempts);
  EXPECT_TRUE(!model.issue_next(now).valid());
}

void test_nonzero_completion_status_retries_the_active_procedure()
{
  Model model;
  const uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
  Model::Request dle = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(dle, now));
  EXPECT_TRUE(model.on_dle_complete(dle.handle, dle.generation, 251U, 251U, now));
  Model::Request phy = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(phy, now));
  EXPECT_TRUE(model.on_phy_complete(phy.handle, phy.generation,
                                    Model::kStatusCommandDisallowed, 1U, 1U, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::NeedPhy);
  EXPECT_TRUE(model.telemetry(0U).confirmed_tx_phy == 0U);
}

void test_permanent_error_fails_the_link()
{
  Model model;
  constexpr uint32_t connected_at = 10U;
  const uint32_t now = connected_at + Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, connected_at));
  const Model::Request request = model.issue_next(now);
  EXPECT_TRUE(model.on_request_status(request, Model::kStatusInvalidParameters, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::Failed);
  EXPECT_TRUE(!model.collection_allowed(0U));
}

void test_missing_completion_degrades_but_keeps_collection_available()
{
  Model model;
  uint32_t now = 10U;
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  now += Model::kCommissioningStartDelayMs;
  const Model::Request request = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  now += Model::kProcedureTimeoutMs;
  EXPECT_TRUE(model.on_timeout(now));
  EXPECT_TRUE(model.telemetry(0U).state == State::Degraded);
  EXPECT_TRUE(model.collection_allowed(0U));
  EXPECT_TRUE(!model.issue_next(now).valid());
}

void test_stale_or_disconnected_completions_are_rejected()
{
  Model model;
  constexpr uint32_t connected_at = 10U;
  const uint32_t now = connected_at + Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, connected_at));
  const Model::Request request = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  EXPECT_TRUE(model.disconnect(0U, request.handle));
  EXPECT_TRUE(!model.on_dle_complete(request.handle, request.generation, 251U, 251U, now));
  EXPECT_TRUE(model.connect(0U, 0x0041U, now));
  EXPECT_TRUE(model.telemetry(0U).generation == request.generation + 1U);
  EXPECT_TRUE(!model.on_dle_complete(request.handle, request.generation, 251U, 251U, now));
}

void test_transfer_preparation_gate_is_source_and_generation_safe()
{
  Model model;
  uint32_t now = 10U;
  for (uint8_t link = 0U; link < Model::kLinkCount; ++link) {
    EXPECT_TRUE(model.connect(link, static_cast<uint16_t>(0x0040U + link), now));
  }
  now += Model::kCommissioningStartDelayMs;
  for (uint8_t link = 0U; link < Model::kLinkCount; ++link) {
    complete_fast_tune(model, link, static_cast<uint16_t>(0x0040U + link),
                       model.telemetry(link).generation, now);
  }

  const uint32_t source_generation = model.telemetry(2U).generation;
  EXPECT_TRUE(model.transfer_preparation_resolved(0U,
                                                  model.telemetry(0U).generation));
  EXPECT_TRUE(model.begin_fast_preparation(2U, source_generation));
  /* A different Ready link must never release source 2's initial credit. */
  EXPECT_TRUE(!model.transfer_preparation_resolved(2U, source_generation));
  const Model::Request fast = model.issue_next(now);
  EXPECT_TRUE(fast.valid() && fast.link == 2U && fast.interval == Model::Interval::Fast);
  EXPECT_TRUE(model.on_request_accepted(fast, now));
  EXPECT_TRUE(!model.transfer_preparation_resolved(2U, source_generation));
  EXPECT_TRUE(model.on_interval_complete(fast.handle, source_generation,
                                         Model::kStatusSuccess,
                                         Model::kFastIntervalMin, now));
  EXPECT_TRUE(model.transfer_preparation_resolved(2U, source_generation));

  EXPECT_TRUE(model.disconnect(2U, fast.handle));
  EXPECT_TRUE(!model.transfer_preparation_resolved(2U, source_generation));
  EXPECT_TRUE(model.connect(2U, 0x0052U, now));
  EXPECT_TRUE(model.telemetry(2U).generation == source_generation + 1U);
  EXPECT_TRUE(!model.transfer_preparation_resolved(2U, source_generation));
  EXPECT_TRUE(!model.transfer_preparation_resolved(2U,
                                                   model.telemetry(2U).generation));
}

void test_transfer_preparation_gate_opens_on_explicit_fallbacks()
{
  {
    Model model;
    uint32_t now = Model::kCommissioningStartDelayMs;
    EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
    const uint32_t generation = model.telemetry(0U).generation;
    complete_fast_tune(model, 0U, 0x0040U, generation, now);
    EXPECT_TRUE(model.begin_fast_preparation(0U, generation));
    const Model::Request fast = model.issue_next(now);
    EXPECT_TRUE(model.on_request_accepted(fast, now));
    now += Model::kProcedureTimeoutMs;
    EXPECT_TRUE(model.on_timeout(now));
    EXPECT_TRUE(model.telemetry(0U).state == State::Degraded);
    EXPECT_TRUE(model.transfer_preparation_resolved(0U, generation));
  }

  {
    Model model;
    uint32_t now = Model::kCommissioningStartDelayMs;
    EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
    const uint32_t generation = model.telemetry(0U).generation;
    complete_fast_tune(model, 0U, 0x0040U, generation, now);
    EXPECT_TRUE(model.begin_fast_preparation(0U, generation));
    const Model::Request fast = model.issue_next(now);
    EXPECT_TRUE(model.on_request_status(fast, Model::kStatusInvalidParameters, now));
    EXPECT_TRUE(model.telemetry(0U).state == State::Failed);
    /* Link tuning failed permanently, but reliable collection must fall back. */
    EXPECT_TRUE(model.transfer_preparation_resolved(0U, generation));
  }
}

void test_fast_preparation_latches_during_initial_commissioning()
{
  Model model;
  uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
  const uint32_t generation = model.telemetry(0U).generation;
  Model::Request request = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::WaitDle);
  EXPECT_TRUE(model.begin_fast_preparation(0U, generation, 6U));
  EXPECT_TRUE(model.on_dle_complete(request.handle, generation, 251U, 251U, now));
  request = model.issue_next(now);
  EXPECT_TRUE(request.procedure == Procedure::Phy);
  EXPECT_TRUE(model.on_request_accepted(request, now));
  EXPECT_TRUE(model.on_phy_complete(request.handle, generation,
                                    Model::kStatusSuccess, 2U, 2U, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::NeedInterval);
  request = model.issue_next(now);
  EXPECT_TRUE(request.valid() && request.interval == Model::Interval::Fast);
  EXPECT_TRUE(request.interval_min == 6U && request.interval_max == 6U);
}

void test_fast_to_slow_interval_uses_the_current_generation_only()
{
  Model model;
  uint32_t now = 10U;
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  now += Model::kCommissioningStartDelayMs;
  const uint32_t generation = model.telemetry(0U).generation;
  complete_fast_tune(model, 0U, 0x0040U, generation, now);
  EXPECT_TRUE(model.begin_fast_preparation(0U, generation));
  const Model::Request fast = model.issue_next(now);
  EXPECT_TRUE(fast.valid());
  EXPECT_TRUE(fast.procedure == Procedure::Interval);
  EXPECT_TRUE(fast.interval == Model::Interval::Fast);
  EXPECT_TRUE(model.on_request_accepted(fast, now));
  EXPECT_TRUE(model.on_interval_complete(0x0040U, generation, Model::kStatusSuccess,
                                         Model::kFastIntervalMin, now));
  EXPECT_TRUE(model.begin_slow_restore(0U, generation));
  const Model::Request slow = model.issue_next(now);
  EXPECT_TRUE(slow.valid());
  EXPECT_TRUE(slow.procedure == Procedure::Interval);
  EXPECT_TRUE(slow.interval == Model::Interval::Slow);
  EXPECT_TRUE(model.on_request_accepted(slow, now));
  EXPECT_TRUE(!model.on_interval_complete(0x0040U, generation + 1U, Model::kStatusSuccess,
                                          Model::kFastIntervalMin, now));
  EXPECT_TRUE(model.on_interval_complete(0x0040U, generation, Model::kStatusSuccess,
                                         Model::kSlowIntervalMin, now));
  EXPECT_TRUE(model.telemetry(0U).state == State::Ready);
  EXPECT_TRUE(model.telemetry(0U).confirmed_interval == Model::kSlowIntervalMin);
}

void test_slow_restore_latches_during_fast_wait_and_runs_next_globally()
{
  Model model;
  uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(3U, 0x0043U, 0U));
  const uint32_t generation = model.telemetry(3U).generation;
  complete_fast_tune(model, 3U, 0x0043U, generation, now);
  EXPECT_TRUE(model.begin_fast_preparation(3U, generation));
  const Model::Request fast = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(fast, now));
  EXPECT_TRUE(model.telemetry(3U).state == State::WaitInterval);

  /* Link 0 is eligible when the fast completion arrives, but the serialized
   * next action must honor source 3's already-latched slow restore first. */
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  EXPECT_TRUE(model.begin_slow_restore(3U, generation));
  now += Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.on_interval_complete(fast.handle, generation,
                                         Model::kStatusSuccess,
                                         Model::kFastIntervalMin, now));
  const Model::Request slow = model.issue_next(now);
  EXPECT_TRUE(slow.valid() && slow.link == 3U);
  EXPECT_TRUE(slow.procedure == Procedure::Interval);
  EXPECT_TRUE(slow.interval == Model::Interval::Slow);
  EXPECT_TRUE(model.on_request_accepted(slow, now));
  EXPECT_TRUE(!model.on_interval_complete(slow.handle, generation + 1U,
                                          Model::kStatusSuccess,
                                          Model::kSlowIntervalMin, now));
  EXPECT_TRUE(model.on_interval_complete(slow.handle, generation,
                                         Model::kStatusSuccess,
                                         Model::kSlowIntervalMin, now));
}

void test_latched_slow_restore_survives_fast_error_and_slow_retry()
{
  Model model;
  uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
  const uint32_t generation = model.telemetry(0U).generation;
  complete_fast_tune(model, 0U, 0x0040U, generation, now);
  EXPECT_TRUE(model.begin_fast_preparation(0U, generation));
  const Model::Request fast = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(fast, now));
  EXPECT_TRUE(model.begin_slow_restore(0U, generation));
  EXPECT_TRUE(model.on_interval_complete(fast.handle, generation,
                                         Model::kStatusInvalidParameters,
                                         Model::kFastIntervalMin, now));

  Model::Request slow = model.issue_next(now);
  EXPECT_TRUE(slow.valid() && slow.interval == Model::Interval::Slow);
  EXPECT_TRUE(model.on_request_status(slow, Model::kStatusControllerBusy, now));
  EXPECT_TRUE(!model.issue_next(now).valid());
  now += Model::kRetryDelayMs;
  slow = model.issue_next(now);
  EXPECT_TRUE(slow.valid() && slow.interval == Model::Interval::Slow);
}

void test_latched_slow_restore_survives_fast_timeout()
{
  Model model;
  uint32_t now = Model::kCommissioningStartDelayMs;
  EXPECT_TRUE(model.connect(0U, 0x0040U, 0U));
  const uint32_t generation = model.telemetry(0U).generation;
  complete_fast_tune(model, 0U, 0x0040U, generation, now);
  EXPECT_TRUE(model.begin_fast_preparation(0U, generation));
  const Model::Request fast = model.issue_next(now);
  EXPECT_TRUE(model.on_request_accepted(fast, now));
  EXPECT_TRUE(model.begin_slow_restore(0U, generation));
  now += Model::kProcedureTimeoutMs;
  EXPECT_TRUE(model.on_timeout(now));
  const Model::Request slow = model.issue_next(now);
  EXPECT_TRUE(slow.valid() && slow.interval == Model::Interval::Slow);
}

void test_supported_fast_intervals_are_snapshotted_per_link()
{
  Model model;
  uint32_t now = 10U;
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  EXPECT_TRUE(model.connect(1U, 0x0041U, now));
  now += Model::kCommissioningStartDelayMs;
  const uint32_t generation0 = model.telemetry(0U).generation;
  const uint32_t generation1 = model.telemetry(1U).generation;
  complete_fast_tune(model, 0U, 0x0040U, generation0, now);
  complete_fast_tune(model, 1U, 0x0041U, generation1, now);

  EXPECT_TRUE(model.begin_fast_preparation(0U, generation0, 6U));
  const Model::Request first = model.issue_next(now);
  EXPECT_TRUE(first.valid());
  EXPECT_TRUE(first.link == 0U);
  EXPECT_TRUE(first.interval_min == 6U && first.interval_max == 6U);
  EXPECT_TRUE(model.on_request_accepted(first, now));

  /* A different link may snapshot the next benchmark value while the global
   * arbiter still owns link 0's completion. It must not mutate that request. */
  EXPECT_TRUE(model.begin_fast_preparation(1U, generation1, 9U));
  EXPECT_TRUE(!model.issue_next(now).valid());
  EXPECT_TRUE(model.on_interval_complete(first.handle, generation0,
                                         Model::kStatusSuccess, 6U, now));
  const Model::Request second = model.issue_next(now);
  EXPECT_TRUE(second.valid());
  EXPECT_TRUE(second.link == 1U);
  EXPECT_TRUE(second.interval_min == 9U && second.interval_max == 9U);
}

void test_invalid_interval_falls_back_and_stale_generation_cannot_change_it()
{
  Model model;
  uint32_t now = 10U;
  EXPECT_TRUE(model.connect(0U, 0x0040U, now));
  now += Model::kCommissioningStartDelayMs;
  const uint32_t old_generation = model.telemetry(0U).generation;
  complete_fast_tune(model, 0U, 0x0040U, old_generation, now);
  EXPECT_TRUE(model.disconnect(0U, 0x0040U));
  EXPECT_TRUE(model.connect(0U, 0x0041U, now));
  const uint32_t generation = model.telemetry(0U).generation;
  EXPECT_TRUE(generation == old_generation + 1U);
  EXPECT_TRUE(!model.begin_fast_preparation(0U, old_generation, 6U));

  now += Model::kCommissioningStartDelayMs;
  complete_fast_tune(model, 0U, 0x0041U, generation, now);
  EXPECT_TRUE(model.begin_fast_preparation(0U, generation, 7U));
  const Model::Request fallback = model.issue_next(now);
  EXPECT_TRUE(fallback.valid());
  EXPECT_TRUE(fallback.interval_min == Model::kFastIntervalMin);
  EXPECT_TRUE(fallback.interval_max == Model::kFastIntervalMax);
}

constexpr bool transition_names_are_distinct()
{
  return static_cast<uint8_t>(State::NeedDle) != static_cast<uint8_t>(State::WaitDle) &&
         static_cast<uint8_t>(State::NeedPhy) != static_cast<uint8_t>(State::WaitPhy) &&
         static_cast<uint8_t>(State::NeedInterval) != static_cast<uint8_t>(State::WaitInterval);
}

constexpr bool compile_time_transition_contract()
{
  Model model{};
  if (!model.connect(0U, 0x0040U, 0U) || !model.connect(1U, 0x0041U, 0U)) {
    return false;
  }
  const uint32_t now = Model::kCommissioningStartDelayMs;
  const Model::Request first = model.issue_next(now);
  if (!first.valid() || first.link != 0U || first.procedure != Procedure::Dle ||
      model.issue_next(now).valid() || !model.on_request_accepted(first, now) ||
      !model.on_dle_complete(first.handle, first.generation, 251U, 251U, now)) {
    return false;
  }
  const Model::Request phy = model.issue_next(now);
  if (!phy.valid() || phy.procedure != Procedure::Phy ||
      !model.on_request_accepted(phy, now) ||
      !model.on_phy_complete(phy.handle, phy.generation, Model::kStatusSuccess, 2U, 2U, now) ||
      model.telemetry(0U).state != State::Ready) {
    return false;
  }
  const Model::Request second = model.issue_next(now);
  if (!second.valid() || second.link != 1U ||
      !model.on_request_status(second, Model::kStatusCommandDisallowed, now) ||
      model.issue_next(now).valid()) {
    return false;
  }
  const Model::Request retry = model.issue_next(now + Model::kRetryDelayMs);
  return retry.valid() && retry.link == 1U && retry.procedure == Procedure::Dle;
}

constexpr bool compile_time_failure_and_generation_contract()
{
  Model model{};
  const uint32_t ready_at = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U)) {
    return false;
  }
  const Model::Request permanent = model.issue_next(ready_at);
  if (!model.on_request_status(permanent, Model::kStatusInvalidParameters, ready_at) ||
      model.telemetry(0U).state != State::Failed || model.collection_allowed(0U)) {
    return false;
  }
  if (!model.connect(0U, 0x0041U, ready_at)) {
    return false;
  }
  const Model::Request delayed = model.issue_next(ready_at + Model::kCommissioningStartDelayMs);
  if (!model.on_request_accepted(delayed, ready_at + Model::kCommissioningStartDelayMs) ||
      !model.disconnect(0U, delayed.handle) ||
      model.on_dle_complete(delayed.handle, delayed.generation, 251U, 251U,
                            ready_at + Model::kCommissioningStartDelayMs)) {
    return false;
  }
  if (!model.connect(0U, 0x0042U, ready_at) ||
      model.telemetry(0U).generation != delayed.generation + 1U) {
    return false;
  }
  const Model::Request timeout = model.issue_next(ready_at + Model::kCommissioningStartDelayMs);
  return model.on_request_accepted(timeout, ready_at + Model::kCommissioningStartDelayMs) &&
         model.on_timeout(ready_at + Model::kCommissioningStartDelayMs +
                          Model::kProcedureTimeoutMs) &&
         model.telemetry(0U).state == State::Degraded && model.collection_allowed(0U);
}

constexpr bool compile_time_transfer_tuning_wire_contract()
{
  uint8_t legacy[TuningWire::kV1Length]{};
  legacy[0] = TuningWire::kCommand;
  legacy[1] = TuningWire::kVersion1;
  const auto v1 = TuningWire::decode(legacy, sizeof(legacy));
  if (!v1.valid || v1.version != TuningWire::kVersion1 ||
      v1.fast_interval != Model::kFastIntervalMin) {
    return false;
  }

  uint8_t extended[TuningWire::kV2Length]{};
  extended[0] = TuningWire::kCommand;
  extended[1] = TuningWire::kVersion2;
  extended[TuningWire::kFastIntervalOffset] = 9U;
  const auto ci9 = TuningWire::decode(extended, sizeof(extended));
  if (!ci9.valid || ci9.version != TuningWire::kVersion2 || ci9.fast_interval != 9U) {
    return false;
  }
  extended[TuningWire::kFastIntervalOffset] = 6U;
  if (TuningWire::decode(extended, sizeof(extended)).fast_interval != 6U) {
    return false;
  }
  extended[TuningWire::kFastIntervalOffset] = 12U;
  if (TuningWire::decode(extended, sizeof(extended)).fast_interval != 12U) {
    return false;
  }
  extended[TuningWire::kFastIntervalOffset] = 7U;
  if (TuningWire::decode(extended, sizeof(extended)).fast_interval != 12U) {
    return false;
  }
  return !TuningWire::decode(legacy, sizeof(legacy) - 1U).valid &&
         !TuningWire::decode(extended, sizeof(extended) - 1U).valid;
}

constexpr bool compile_time_per_link_interval_race_contract()
{
  Model model{};
  uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U) || !model.connect(1U, 0x0041U, 0U)) {
    return false;
  }
  for (uint8_t link = 0U; link < 2U; ++link) {
    const uint16_t handle = static_cast<uint16_t>(0x0040U + link);
    const uint32_t generation = model.telemetry(link).generation;
    Model::Request request = model.issue_next(now);
    if (request.link != link || request.procedure != Procedure::Dle ||
        !model.on_request_accepted(request, now) ||
        !model.on_dle_complete(handle, generation, 251U, 251U, now)) {
      return false;
    }
    request = model.issue_next(now);
    if (request.link != link || request.procedure != Procedure::Phy ||
        !model.on_request_accepted(request, now) ||
        !model.on_phy_complete(handle, generation, Model::kStatusSuccess, 2U, 2U, now)) {
      return false;
    }
  }
  const uint32_t generation0 = model.telemetry(0U).generation;
  const uint32_t generation1 = model.telemetry(1U).generation;
  if (!model.begin_fast_preparation(0U, generation0, 6U)) {
    return false;
  }
  const Model::Request ci6 = model.issue_next(now);
  if (ci6.link != 0U || ci6.interval_min != 6U || ci6.interval_max != 6U ||
      !model.on_request_accepted(ci6, now) ||
      !model.begin_fast_preparation(1U, generation1, 9U) ||
      model.issue_next(now).valid() ||
      !model.on_interval_complete(ci6.handle, generation0, Model::kStatusSuccess, 6U, now)) {
    return false;
  }
  const Model::Request ci9 = model.issue_next(now);
  if (ci9.link != 1U || ci9.interval_min != 9U || ci9.interval_max != 9U ||
      !model.disconnect(1U, ci9.handle) || !model.connect(1U, 0x0042U, now)) {
    return false;
  }
  return !model.begin_fast_preparation(1U, generation1, 6U) &&
         model.telemetry(1U).generation == generation1 + 1U;
}

constexpr bool compile_time_transfer_gate_and_pending_restore_contract()
{
  Model model{};
  uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U)) return false;
  const uint32_t generation = model.telemetry(0U).generation;
  Model::Request request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_dle_complete(request.handle, generation, 251U, 251U, now)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_phy_complete(request.handle, generation, Model::kStatusSuccess,
                             2U, 2U, now) ||
      !model.transfer_preparation_resolved(0U, generation) ||
      !model.begin_fast_preparation(0U, generation)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      model.transfer_preparation_resolved(0U, generation) ||
      !model.begin_slow_restore(0U, generation) ||
      !model.on_interval_complete(request.handle, generation, Model::kStatusSuccess,
                                  Model::kFastIntervalMin, now)) return false;
  const Model::Request slow = model.issue_next(now);
  return slow.valid() && slow.interval == Model::Interval::Slow &&
         slow.generation == generation;
}

constexpr bool compile_time_fast_latch_during_commissioning_contract()
{
  Model model{};
  const uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U)) return false;
  const uint32_t generation = model.telemetry(0U).generation;
  Model::Request request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.begin_fast_preparation(0U, generation, 6U) ||
      !model.on_dle_complete(request.handle, generation, 251U, 251U, now)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_phy_complete(request.handle, generation, Model::kStatusSuccess,
                             2U, 2U, now)) return false;
  request = model.issue_next(now);
  return request.valid() && request.procedure == Procedure::Interval &&
         request.interval == Model::Interval::Fast && request.interval_min == 6U;
}

constexpr bool compile_time_pending_restore_error_contract()
{
  Model model{};
  uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U)) return false;
  const uint32_t generation = model.telemetry(0U).generation;
  Model::Request request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_dle_complete(request.handle, generation, 251U, 251U, now)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_phy_complete(request.handle, generation, Model::kStatusSuccess,
                             2U, 2U, now) ||
      !model.begin_fast_preparation(0U, generation)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.begin_slow_restore(0U, generation) ||
      !model.on_interval_complete(request.handle, generation,
                                  Model::kStatusInvalidParameters,
                                  Model::kFastIntervalMin, now)) return false;
  const Model::Request slow = model.issue_next(now);
  return slow.valid() && slow.procedure == Procedure::Interval &&
         slow.interval == Model::Interval::Slow && slow.generation == generation;
}

constexpr bool compile_time_transfer_gate_fallback_contract()
{
  Model model{};
  uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(2U, 0x0042U, 0U)) return false;
  const uint32_t generation = model.telemetry(2U).generation;
  Model::Request request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      model.transfer_preparation_resolved(2U, generation)) return false;
  now += Model::kProcedureTimeoutMs;
  if (!model.on_timeout(now) ||
      !model.transfer_preparation_resolved(2U, generation) ||
      !model.disconnect(2U, request.handle) ||
      model.transfer_preparation_resolved(2U, generation) ||
      !model.connect(2U, 0x0052U, now)) return false;
  const uint32_t replacement_generation = model.telemetry(2U).generation;
  request = model.issue_next(now + Model::kCommissioningStartDelayMs);
  return replacement_generation == generation + 1U && request.link == 2U &&
         model.on_request_status(request, Model::kStatusInvalidParameters,
                                 now + Model::kCommissioningStartDelayMs) &&
         model.transfer_preparation_resolved(2U, replacement_generation) &&
         !model.transfer_preparation_resolved(2U, generation);
}

constexpr bool compile_time_fast_reselection_during_slow_restore_contract()
{
  Model model{};
  uint32_t now = Model::kCommissioningStartDelayMs;
  if (!model.connect(0U, 0x0040U, 0U)) return false;
  const uint32_t generation = model.telemetry(0U).generation;
  Model::Request request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_dle_complete(request.handle, generation, 251U, 251U, now)) return false;
  request = model.issue_next(now);
  if (!model.on_request_accepted(request, now) ||
      !model.on_phy_complete(request.handle, generation, Model::kStatusSuccess,
                             2U, 2U, now) ||
      !model.begin_slow_restore(0U, generation)) return false;
  const Model::Request slow = model.issue_next(now);
  if (!model.on_request_accepted(slow, now) ||
      !model.begin_fast_preparation(0U, generation, 6U) ||
      !model.on_interval_complete(slow.handle, generation, Model::kStatusSuccess,
                                  Model::kSlowIntervalMin, now) ||
      model.transfer_preparation_resolved(0U, generation)) return false;
  const Model::Request fast = model.issue_next(now);
  return fast.valid() && fast.procedure == Procedure::Interval &&
         fast.interval == Model::Interval::Fast && fast.interval_min == 6U;
}

static_assert(transition_names_are_distinct(),
              "The compile-time transition-state contract must stay distinct");
static_assert(compile_time_transition_contract(),
              "The global arbiter must serialize completion-driven retries");
static_assert(compile_time_failure_and_generation_contract(),
              "Timeout degradation and stale completion rejection must hold");
static_assert(compile_time_transfer_tuning_wire_contract(),
              "B5 v1 defaults and B5 v2 interval selection must stay wire compatible");
static_assert(compile_time_per_link_interval_race_contract(),
              "Per-link interval snapshots must survive global arbitration and reject stale generations");
static_assert(compile_time_transfer_gate_and_pending_restore_contract(),
              "Initial credit gating and pending slow restore must stay generation safe");
static_assert(compile_time_fast_latch_during_commissioning_contract(),
              "A source selected during commissioning must still receive fast preparation");
static_assert(compile_time_pending_restore_error_contract(),
              "A fast error must hand the global procedure slot to the pending slow restore");
static_assert(compile_time_transfer_gate_fallback_contract(),
              "The source gate must allow explicit fallback but reject disconnects and stale generations");
static_assert(compile_time_fast_reselection_during_slow_restore_contract(),
              "A reselected source must finish fast preparation after an in-flight slow restore");

}  // namespace

int main()
{
  test_serialized_four_link_arbitration();
  test_transient_error_retries_without_releasing_the_global_arbiter();
  test_transient_failures_are_bounded_and_degrade();
  test_nonzero_completion_status_retries_the_active_procedure();
  test_permanent_error_fails_the_link();
  test_missing_completion_degrades_but_keeps_collection_available();
  test_stale_or_disconnected_completions_are_rejected();
  test_transfer_preparation_gate_is_source_and_generation_safe();
  test_transfer_preparation_gate_opens_on_explicit_fallbacks();
  test_fast_preparation_latches_during_initial_commissioning();
  test_fast_to_slow_interval_uses_the_current_generation_only();
  test_slow_restore_latches_during_fast_wait_and_runs_next_globally();
  test_latched_slow_restore_survives_fast_error_and_slow_retry();
  test_latched_slow_restore_survives_fast_timeout();
  test_supported_fast_intervals_are_snapshotted_per_link();
  test_invalid_interval_falls_back_and_stale_generation_cannot_change_it();

  if (failures != 0) {
    std::cerr << failures << " link tune state test(s) failed\n";
    return 1;
  }
  std::cout << "link tune state tests passed\n";
  return 0;
}

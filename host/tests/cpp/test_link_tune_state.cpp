#include <cstdint>
#include <iostream>

#include <exo/ble/link_tune_state.h>

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

static_assert(transition_names_are_distinct(),
              "The compile-time transition-state contract must stay distinct");
static_assert(compile_time_transition_contract(),
              "The global arbiter must serialize completion-driven retries");
static_assert(compile_time_failure_and_generation_contract(),
              "Timeout degradation and stale completion rejection must hold");

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
  test_fast_to_slow_interval_uses_the_current_generation_only();

  if (failures != 0) {
    std::cerr << failures << " link tune state test(s) failed\n";
    return 1;
  }
  std::cout << "link tune state tests passed\n";
  return 0;
}

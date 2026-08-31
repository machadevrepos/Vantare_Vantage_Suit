#pragma once

#include <array>
#include <cstdint>

#include <exo/protocol/record_transfer_tuning.h>

namespace exo {

/*
 * Completion-driven commissioning for the Master-to-Node links.  The model is
 * deliberately independent of the STM32 BLE stack: the client issues the
 * returned request, then reports its command status and eventual LL event back
 * here.  That keeps one global LL procedure in flight across every leaf.
 */
class LinkTuneState {
 public:
  enum class State : uint8_t {
    NeedDle,
    WaitDle,
    NeedPhy,
    WaitPhy,
    NeedInterval,
    WaitInterval,
    Ready,
    Degraded,
    Failed,
  };

  enum class Procedure : uint8_t {
    None,
    Dle,
    Phy,
    Interval,
  };

  enum class Interval : uint8_t {
    Fast,
    Slow,
    Parked,
  };

  static constexpr uint8_t kLinkCount = 4U;
  static constexpr uint16_t kInvalidHandle = 0xFFFFU;
  static constexpr uint16_t kRequestedDleOctets = 251U;
  static constexpr uint16_t kRequestedDleTimeUs = 0x0848U;
  static constexpr uint8_t kRequestedPhy = 2U;
  static constexpr uint16_t kFastIntervalMin =
      RecordTransferTuningWire::kDefaultFastInterval;
  static constexpr uint16_t kFastIntervalMax = kFastIntervalMin;
  static constexpr uint16_t kSlowIntervalMin = 0x0018U;
  static constexpr uint16_t kSlowIntervalMax = 0x0028U;
  static constexpr uint16_t kBulkInterval =
      RecordTransferTuningWire::kBulkFastInterval;
  static constexpr uint16_t kParkedInterval = 0x0090U; /* 180 ms */
  static constexpr uint16_t kBulkMinCeLength = 0x0000U;
  static constexpr uint16_t kBulkMaxCeLength = 0x0030U; /* 30 ms */
  static_assert(kBulkMaxCeLength <= (2U * kBulkInterval),
                "bulk connection event cannot exceed its interval");
  static_assert((kParkedInterval % kBulkInterval) == 0U,
                "parked interval must remain an exact bulk-interval multiple");
  static constexpr uint32_t kRetryDelayMs = 50U;
  static constexpr uint32_t kCommissioningStartDelayMs = 600U;
  static constexpr uint32_t kProcedureTimeoutMs = 1500U;
  static constexpr uint8_t kMaxTransientAttempts = 4U;
  static constexpr uint8_t kStatusSuccess = 0x00U;
  static constexpr uint8_t kStatusCommandDisallowed = 0x0CU;
  static constexpr uint8_t kStatusControllerBusy = 0x3AU;
  static constexpr uint8_t kStatusNewIntervalFailed = 0x84U;
  static constexpr uint8_t kStatusIntervalTooLarge = 0x85U;
  static constexpr uint8_t kStatusLengthFailed = 0x86U;
  static constexpr uint8_t kStatusInvalidParameters = 0x12U;
  static constexpr uint8_t kStatusNotRequested = 0xFFU;
  static constexpr uint8_t kStatusTimeout = 0xFEU;
  /* STM32WB grants a connection interval only when it is an integer multiple
   * or submultiple of the current virtual anchor period. 0x84/0x85 mean "no
   * such multiple/submultiple in the requested range" (0x86: not enough radio
   * time), so every one of them is a scheduler rejection the ladder must
   * retry, never a terminal link failure. Observed on this fleet: with a
   * 40 ms anchor (initial links), 15 ms exact is ungrantable (0x84) and a
   * 30-50 ms request can be ungrantable (0x85) depending on the other slots,
   * while the wide 7.5-30 ms range always confirmed at 20 ms (= 40/2). The
   * ladder therefore walks: exact -> widened -> 10-15 ms submultiple window
   * (10 ms = 40/4, the only sub-20 ms interval a 40 ms anchor can grant) ->
   * the proven wide 7.5-30 ms range as the guaranteed-grantable floor. */
  static constexpr uint16_t kFastIntervalFallbackSpan = 6U;
  static constexpr uint16_t kFastIntervalFallbackMax = kSlowIntervalMin;
  static constexpr uint8_t kFastIntervalFallbackLevels = 3U;
  static constexpr uint16_t kSubMultipleIntervalMin = 0x0008U; /* 10 ms */
  static constexpr uint16_t kSubMultipleIntervalMax = 0x000CU; /* 15 ms */

  struct Request {
    uint8_t link = 0xFFU;
    uint16_t handle = kInvalidHandle;
    uint32_t generation = 0U;
    Procedure procedure = Procedure::None;
    Interval interval = Interval::Fast;
    uint16_t dle_octets = 0U;
    uint16_t dle_time_us = 0U;
    uint8_t tx_phy = 0U;
    uint8_t rx_phy = 0U;
    uint16_t interval_min = 0U;
    uint16_t interval_max = 0U;
    uint16_t min_ce_length = 0U;
    uint16_t max_ce_length = 0U;

    constexpr bool valid() const { return procedure != Procedure::None; }
  };

  struct Telemetry {
    State state = State::Failed;
    uint16_t handle = kInvalidHandle;
    uint32_t generation = 0U;
    uint16_t requested_dle_octets = kRequestedDleOctets;
    uint16_t confirmed_dle_tx_octets = 0U;
    uint16_t confirmed_dle_rx_octets = 0U;
    uint8_t requested_tx_phy = kRequestedPhy;
    uint8_t requested_rx_phy = kRequestedPhy;
    uint8_t confirmed_tx_phy = 0U;
    uint8_t confirmed_rx_phy = 0U;
    uint16_t requested_interval_min = kFastIntervalMin;
    uint16_t requested_interval_max = kFastIntervalMax;
    uint16_t requested_min_ce_length = 0U;
    uint16_t requested_max_ce_length = 0U;
    uint16_t confirmed_interval = 0U;
    uint32_t preparation_duration_ms = 0U;
    uint8_t retries = 0U;
    uint8_t status = kStatusNotRequested;
  };

  constexpr LinkTuneState() = default;

  constexpr bool connect(uint8_t link, uint16_t handle, uint32_t now_ms) {
    if (link >= kLinkCount || handle == kInvalidHandle) {
      return false;
    }
    Entry &entry = links_[link];
    if (active_.valid() && active_.link == link) {
      active_ = Request{};
      active_accepted_ = false;
    }
    entry.connected = true;
    entry.telemetry = Telemetry{};
    entry.telemetry.handle = handle;
    entry.telemetry.generation = entry.next_generation++;
    entry.telemetry.state = State::NeedDle;
    entry.connected_at_ms = now_ms;
    entry.retry_at_ms = now_ms + kCommissioningStartDelayMs;
    entry.interval = Interval::Slow;
    entry.fast_interval = kFastIntervalMin;
    entry.interval_fallback_level = 0U;
    entry.had_unconfirmed_procedure = false;
    entry.fast_preparation_pending = false;
    entry.slow_restore_pending = false;
    entry.park_preparation_pending = false;
    return true;
  }

  constexpr bool disconnect(uint8_t link, uint16_t handle) {
    if (link >= kLinkCount) {
      return false;
    }
    Entry &entry = links_[link];
    if (!entry.connected || entry.telemetry.handle != handle) {
      return false;
    }
    if (active_.valid() && active_.link == link && active_.handle == handle) {
      active_ = Request{};
      active_accepted_ = false;
    }
    entry.connected = false;
    entry.telemetry.state = State::Failed;
    entry.fast_preparation_pending = false;
    entry.slow_restore_pending = false;
    entry.park_preparation_pending = false;
    return true;
  }

  constexpr Request issue_next(uint32_t now_ms) {
    if (active_.valid()) {
      return Request{};
    }
    /* A source completion that raced its fast-interval procedure already owns
     * the next serialized action: restore that link before commissioning work
     * on a lower-numbered slot can overtake it. */
    for (uint8_t link = 0U; link < kLinkCount; ++link) {
      Entry &entry = links_[link];
      if (!entry.slow_restore_pending || !entry.connected ||
          now_ms < entry.retry_at_ms || !needs_request(entry.telemetry.state)) {
        continue;
      }
      active_ = make_request(link, entry);
      active_accepted_ = false;
      return active_;
    }
    for (uint8_t link = 0U; link < kLinkCount; ++link) {
      Entry &entry = links_[link];
      if (!entry.connected || now_ms < entry.retry_at_ms || !needs_request(entry.telemetry.state)) {
        continue;
      }
      active_ = make_request(link, entry);
      active_accepted_ = false;
      return active_;
    }
    return Request{};
  }

  constexpr bool on_request_accepted(const Request &request, uint32_t now_ms) {
    if (!matches_active(request)) {
      return false;
    }
    Entry &entry = links_[request.link];
    entry.telemetry.status = kStatusSuccess;
    entry.telemetry.state = wait_state(request.procedure);
    entry.deadline_ms = now_ms + kProcedureTimeoutMs;
    active_accepted_ = true;
    return true;
  }

  constexpr bool on_request_status(const Request &request, uint8_t status, uint32_t now_ms) {
    if (status == kStatusSuccess) {
      return on_request_accepted(request, now_ms);
    }
    if (!matches_active(request)) {
      return false;
    }
    /* Completion-status callbacks pass active_ itself. Snapshot it before
     * clearing the global procedure slot so recovery decisions remain tied to
     * the procedure and interval that actually failed. */
    const Request completed_request = request;
    Entry &entry = links_[request.link];
    entry.telemetry.status = status;
    active_ = Request{};
    active_accepted_ = false;
    const bool interval_schedule_rejected =
        completed_request.procedure == Procedure::Interval &&
        (status == kStatusNewIntervalFailed || status == kStatusIntervalTooLarge ||
         status == kStatusLengthFailed);
    if (is_transient(status) || interval_schedule_rejected) {
      ++entry.telemetry.retries;
      if (interval_schedule_rejected &&
          completed_request.interval == Interval::Fast &&
          entry.interval_fallback_level < kFastIntervalFallbackLevels) {
        ++entry.interval_fallback_level;
      }
      if (entry.telemetry.retries >= kMaxTransientAttempts) {
        entry.telemetry.state = State::Degraded;
        entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
      } else {
        entry.telemetry.state = need_state(completed_request.procedure);
        entry.retry_at_ms = now_ms + kRetryDelayMs;
      }
    } else {
      entry.telemetry.state = State::Failed;
    }
    const bool terminal_procedure = entry.telemetry.state == State::Degraded ||
                                    entry.telemetry.state == State::Failed;
    if (completed_request.procedure == Procedure::Dle && terminal_procedure &&
        entry.fast_preparation_pending) {
      entry.had_unconfirmed_procedure = true;
      entry.telemetry.state = State::NeedPhy;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Dle && terminal_procedure &&
               entry.park_preparation_pending) {
      entry.had_unconfirmed_procedure = true;
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Phy && terminal_procedure &&
               entry.fast_preparation_pending) {
      entry.had_unconfirmed_procedure = true;
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Phy && terminal_procedure &&
               entry.park_preparation_pending) {
      entry.had_unconfirmed_procedure = true;
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Interval &&
        completed_request.interval != Interval::Parked &&
        entry.park_preparation_pending) {
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Interval &&
        entry.fast_preparation_pending &&
        (completed_request.interval != Interval::Fast ||
         completed_request.interval_min != entry.fast_interval)) {
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Interval &&
        (completed_request.interval == Interval::Fast ||
         completed_request.interval == Interval::Parked) &&
        entry.slow_restore_pending) {
      /* The transfer ended while the fast procedure was in flight. Its error
       * resolves that request; the next request must be the latched restore,
       * not a retry of an interval the source no longer needs. */
      entry.interval = Interval::Slow;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (completed_request.procedure == Procedure::Interval &&
               completed_request.interval == Interval::Slow &&
               (entry.telemetry.state == State::Degraded ||
                entry.telemetry.state == State::Failed)) {
      entry.slow_restore_pending = false;
    } else if (completed_request.procedure == Procedure::Interval &&
               completed_request.interval == Interval::Parked &&
               (entry.telemetry.state == State::Degraded ||
                entry.telemetry.state == State::Failed)) {
      entry.park_preparation_pending = false;
    }
    return true;
  }

  constexpr bool on_dle_complete(uint16_t handle, uint32_t generation,
                       uint16_t tx_octets, uint16_t rx_octets, uint32_t now_ms) {
    if (!matches_completion(handle, generation, Procedure::Dle)) {
      return false;
    }
    Entry &entry = links_[active_.link];
    entry.telemetry.confirmed_dle_tx_octets = tx_octets;
    entry.telemetry.confirmed_dle_rx_octets = rx_octets;
    entry.telemetry.status = kStatusSuccess;
    entry.telemetry.state = State::NeedPhy;
    complete_active();
    (void)now_ms;
    return true;
  }

  constexpr bool on_phy_complete(uint16_t handle, uint32_t generation, uint8_t status,
                       uint8_t tx_phy, uint8_t rx_phy, uint32_t now_ms) {
    if (status != kStatusSuccess) {
      return on_completion_status(handle, generation, Procedure::Phy, status, now_ms);
    }
    if (!matches_completion(handle, generation, Procedure::Phy)) {
      return false;
    }
    Entry &entry = links_[active_.link];
    entry.telemetry.confirmed_tx_phy = tx_phy;
    entry.telemetry.confirmed_rx_phy = rx_phy;
    entry.telemetry.status = kStatusSuccess;
    if (entry.fast_preparation_pending) {
      entry.interval = Interval::Fast;
      entry.telemetry.state = State::NeedInterval;
    } else if (entry.park_preparation_pending) {
      entry.interval = Interval::Parked;
      entry.telemetry.state = State::NeedInterval;
    } else {
      entry.telemetry.state = entry.had_unconfirmed_procedure ?
          State::Degraded : State::Ready;
    }
    entry.fast_preparation_pending = false;
    entry.park_preparation_pending = false;
    entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
    complete_active();
    (void)now_ms;
    return true;
  }

  constexpr bool on_interval_complete(uint16_t handle, uint32_t generation, uint8_t status,
                            uint16_t interval, uint32_t now_ms) {
    if (status != kStatusSuccess) {
      return on_completion_status(handle, generation, Procedure::Interval, status, now_ms);
    }
    if (!matches_completion(handle, generation, Procedure::Interval)) {
      return false;
    }
    Entry &entry = links_[active_.link];
    const Interval completed_interval = active_.interval;
    const uint16_t completed_interval_min = active_.interval_min;
    entry.telemetry.confirmed_interval = interval;
    entry.telemetry.status = kStatusSuccess;
    entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
    complete_active();
    if (completed_interval != Interval::Parked && entry.park_preparation_pending) {
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if ((completed_interval == Interval::Fast ||
                completed_interval == Interval::Parked) &&
               entry.slow_restore_pending) {
      entry.interval = Interval::Slow;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (entry.fast_preparation_pending &&
               (completed_interval != Interval::Fast ||
                completed_interval_min != entry.fast_interval)) {
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else {
      entry.telemetry.state = entry.had_unconfirmed_procedure ?
          State::Degraded : State::Ready;
      if (completed_interval == Interval::Slow) {
        entry.slow_restore_pending = false;
      } else if (completed_interval == Interval::Parked) {
        entry.park_preparation_pending = false;
      }
    }
    return true;
  }

  constexpr bool on_timeout(uint32_t now_ms) {
    if (!active_.valid() || !active_accepted_ || now_ms < links_[active_.link].deadline_ms) {
      return false;
    }
    Entry &entry = links_[active_.link];
    const Procedure timed_out_procedure = active_.procedure;
    const Interval timed_out_interval = active_.interval;
    const uint16_t timed_out_interval_min = active_.interval_min;
    entry.telemetry.status = kStatusTimeout;
    entry.telemetry.state = State::Degraded;
    entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
    complete_active();
    if (timed_out_procedure == Procedure::Dle) {
      /* A peer may already have negotiated DLE, in which case STM32WB can
       * accept our command without emitting another local change event. Keep
       * DLE truthfully unconfirmed. A later active-transfer preparation will
       * still attempt PHY before changing the connection interval. */
      entry.had_unconfirmed_procedure = true;
      if (entry.fast_preparation_pending) {
        entry.telemetry.state = State::NeedPhy;
        entry.retry_at_ms = now_ms;
      } else if (entry.park_preparation_pending) {
        entry.interval = Interval::Parked;
        entry.park_preparation_pending = false;
        entry.telemetry.state = State::NeedInterval;
        entry.retry_at_ms = now_ms;
      }
    } else if (timed_out_procedure == Procedure::Phy &&
               entry.fast_preparation_pending) {
      /* The adapter gets one final read-PHY opportunity before calling this
       * timeout. If readback is unavailable, continue to interval preparation
       * while retaining Degraded as the final truthful link state. */
      entry.had_unconfirmed_procedure = true;
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (timed_out_procedure == Procedure::Phy &&
               entry.park_preparation_pending) {
      entry.had_unconfirmed_procedure = true;
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (timed_out_procedure == Procedure::Interval &&
        entry.fast_preparation_pending &&
        (timed_out_interval != Interval::Fast ||
         timed_out_interval_min != entry.fast_interval)) {
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (timed_out_procedure == Procedure::Interval &&
        timed_out_interval != Interval::Parked && entry.park_preparation_pending) {
      entry.interval = Interval::Parked;
      entry.park_preparation_pending = false;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (timed_out_procedure == Procedure::Interval &&
        (timed_out_interval == Interval::Fast ||
         timed_out_interval == Interval::Parked) && entry.slow_restore_pending) {
      entry.interval = Interval::Slow;
      entry.telemetry.state = State::NeedInterval;
      entry.retry_at_ms = now_ms;
    } else if (timed_out_procedure == Procedure::Interval &&
               timed_out_interval == Interval::Slow) {
      entry.slow_restore_pending = false;
    } else if (timed_out_procedure == Procedure::Interval &&
               timed_out_interval == Interval::Parked) {
      entry.park_preparation_pending = false;
    }
    return true;
  }

  constexpr bool begin_slow_restore(uint8_t link, uint32_t generation) {
    if (link >= kLinkCount) {
      return false;
    }
    Entry &entry = links_[link];
    if (!entry.connected || entry.telemetry.generation != generation) {
      return false;
    }
    entry.telemetry.retries = 0U;
    if (active_.valid() && active_.link == link) {
      if (active_.procedure != Procedure::Interval ||
          entry.telemetry.state != State::WaitInterval) {
        return false;
      }
      entry.interval = Interval::Slow;
      entry.slow_restore_pending = true;
      entry.park_preparation_pending = false;
      return true;
    }
    if (entry.telemetry.state == State::NeedDle || entry.telemetry.state == State::WaitDle ||
        entry.telemetry.state == State::NeedPhy || entry.telemetry.state == State::WaitPhy) {
      /* The link is still at its idle interval; cancel any fast-after-commission
       * intent because the source completed before that work became useful. */
      entry.fast_preparation_pending = false;
      entry.interval = Interval::Slow;
      entry.park_preparation_pending = false;
      return true;
    }
    if (entry.telemetry.state == State::NeedInterval) {
      entry.interval = Interval::Slow;
      entry.slow_restore_pending = true;
      entry.fast_preparation_pending = false;
      entry.park_preparation_pending = false;
      entry.retry_at_ms = 0U;
      return true;
    }
    if (entry.telemetry.state != State::Ready && entry.telemetry.state != State::Degraded &&
        entry.telemetry.state != State::Failed) {
      return false;
    }
    entry.interval = Interval::Slow;
    entry.slow_restore_pending = true;
    entry.fast_preparation_pending = false;
    entry.park_preparation_pending = false;
    entry.telemetry.state = State::NeedInterval;
    entry.retry_at_ms = 0U;
    return true;
  }

  constexpr bool begin_fast_preparation(uint8_t link, uint32_t generation) {
    return begin_fast_preparation(link, generation, kFastIntervalMin);
  }

  constexpr bool begin_fast_preparation(uint8_t link, uint32_t generation,
                                        uint8_t fast_interval) {
    if (link >= kLinkCount) {
      return false;
    }
    Entry &entry = links_[link];
    if (!entry.connected || entry.telemetry.generation != generation) {
      return false;
    }
    entry.fast_interval = RecordTransferTuningWire::sanitize_fast_interval(fast_interval);
    entry.interval_fallback_level = 0U;
    entry.telemetry.retries = 0U;
    entry.park_preparation_pending = false;
    if (active_.valid() && active_.link == link) {
      if (active_.procedure == Procedure::Interval) {
        if (active_.interval != Interval::Fast ||
            active_.interval_min != entry.fast_interval) {
          entry.fast_preparation_pending = true;
          entry.slow_restore_pending = false;
        }
        return true;
      }
      if (active_.procedure != Procedure::Dle && active_.procedure != Procedure::Phy) {
        return false;
      }
      entry.fast_preparation_pending = true;
      return true;
    }
    if (entry.telemetry.state == State::NeedDle || entry.telemetry.state == State::WaitDle ||
        entry.telemetry.state == State::NeedPhy || entry.telemetry.state == State::WaitPhy) {
      entry.fast_preparation_pending = true;
      return true;
    }
    if (entry.telemetry.state == State::NeedInterval) {
      entry.interval = Interval::Fast;
      entry.fast_preparation_pending = false;
      entry.slow_restore_pending = false;
      entry.retry_at_ms = 0U;
      return true;
    }
    /* Failed is accepted here on purpose: it marks a completed preparation
     * attempt whose interval did not change, not a dead link. Refusing it
     * would leave a connected link stuck at its initial interval for the
     * whole upload (observed: NODE3/NODE4 0x85 on the baseline slow request
     * hard-failed the link and the transfer then ran at half speed). */
    if (entry.telemetry.state != State::Ready && entry.telemetry.state != State::Degraded &&
        entry.telemetry.state != State::Failed) {
      return false;
    }
    if (entry.telemetry.state == State::Degraded &&
        entry.telemetry.confirmed_tx_phy == 0U &&
        entry.telemetry.confirmed_rx_phy == 0U) {
      entry.fast_preparation_pending = true;
      entry.slow_restore_pending = false;
      entry.telemetry.state = State::NeedPhy;
      entry.retry_at_ms = 0U;
      return true;
    }
    entry.interval = Interval::Fast;
    entry.fast_preparation_pending = false;
    entry.slow_restore_pending = false;
    entry.telemetry.state = State::NeedInterval;
    entry.retry_at_ms = 0U;
    return true;
  }

  /* Park an inactive leaf while a different source owns the sequential bulk
   * upload. The request can be latched behind commissioning or an interval
   * procedure already owned by the global LL arbiter. */
  constexpr bool begin_park_preparation(uint8_t link, uint32_t generation) {
    if (link >= kLinkCount) {
      return false;
    }
    Entry &entry = links_[link];
    if (!entry.connected || entry.telemetry.generation != generation) {
      return false;
    }
    entry.telemetry.retries = 0U;
    entry.fast_preparation_pending = false;
    entry.slow_restore_pending = false;
    if (active_.valid() && active_.link == link) {
      if (active_.procedure == Procedure::Interval) {
        entry.park_preparation_pending = active_.interval != Interval::Parked;
        return true;
      }
      if (active_.procedure != Procedure::Dle && active_.procedure != Procedure::Phy) {
        return false;
      }
      entry.park_preparation_pending = true;
      return true;
    }
    if (entry.telemetry.state == State::NeedDle || entry.telemetry.state == State::WaitDle ||
        entry.telemetry.state == State::NeedPhy || entry.telemetry.state == State::WaitPhy) {
      entry.park_preparation_pending = true;
      return true;
    }
    entry.interval = Interval::Parked;
    entry.park_preparation_pending = false;
    entry.telemetry.state = State::NeedInterval;
    entry.retry_at_ms = 0U;
    return true;
  }

  constexpr const Telemetry &telemetry(uint8_t link) const {
    return links_[link].telemetry;
  }

  constexpr bool collection_allowed(uint8_t link) const {
    return link < kLinkCount && (links_[link].telemetry.state == State::Ready ||
                                  links_[link].telemetry.state == State::Degraded);
  }

  /* The initial credit grant is tied to the exact connection generation that
   * was selected for this upload. Ready is confirmed success; Degraded/Failed
   * are explicit tuning fallbacks. A disconnected generation never qualifies,
   * even though disconnect telemetry is Failed. */
  constexpr bool transfer_preparation_resolved(uint8_t link,
                                                uint32_t generation) const {
    if (link >= kLinkCount) {
      return false;
    }
    const Entry &entry = links_[link];
    return entry.connected && entry.telemetry.generation == generation &&
           (entry.telemetry.state == State::Ready ||
            entry.telemetry.state == State::Degraded ||
            entry.telemetry.state == State::Failed);
  }

  constexpr Request active_request() const { return active_; }
  constexpr bool active_timeout_due(uint32_t now_ms) const {
    return active_.valid() && active_accepted_ &&
           now_ms >= links_[active_.link].deadline_ms;
  }

  constexpr bool interval_profile_resolved(uint8_t link, uint32_t generation,
                                           Interval interval) const {
    if (link >= kLinkCount) {
      return false;
    }
    const Entry &entry = links_[link];
    return entry.connected && entry.telemetry.generation == generation &&
           entry.interval == interval && !entry.park_preparation_pending &&
           (entry.telemetry.state == State::Ready ||
            entry.telemetry.state == State::Degraded ||
            entry.telemetry.state == State::Failed);
  }

 private:
  struct Entry {
    Telemetry telemetry{};
    uint32_t next_generation = 1U;
    uint32_t connected_at_ms = 0U;
    uint32_t retry_at_ms = 0U;
    uint32_t deadline_ms = 0U;
    Interval interval = Interval::Fast;
    uint16_t fast_interval = kFastIntervalMin;
    uint8_t interval_fallback_level = 0U;
    bool had_unconfirmed_procedure = false;
    bool fast_preparation_pending = false;
    bool slow_restore_pending = false;
    bool park_preparation_pending = false;
    bool connected = false;
  };

  static constexpr bool needs_request(State state) {
    return state == State::NeedDle || state == State::NeedPhy || state == State::NeedInterval;
  }

  static constexpr State need_state(Procedure procedure) {
    return procedure == Procedure::Dle ? State::NeedDle :
           procedure == Procedure::Phy ? State::NeedPhy : State::NeedInterval;
  }

  static constexpr State wait_state(Procedure procedure) {
    return procedure == Procedure::Dle ? State::WaitDle :
           procedure == Procedure::Phy ? State::WaitPhy : State::WaitInterval;
  }

  static constexpr bool is_transient(uint8_t status) {
    return status == kStatusCommandDisallowed || status == kStatusControllerBusy;
  }

  static constexpr uint16_t interval_min(Interval interval, uint16_t fast_interval,
                                         uint8_t fallback_level) {
    if (interval == Interval::Fast && fallback_level >= kFastIntervalFallbackLevels) {
      /* Final bulk fallback is the previously proven 7.5-30 ms scheduler
       * range, which confirmed at 20 ms on the current hardware. */
      return kFastIntervalMin;
    }
    if (interval == Interval::Fast && fallback_level == 2U) {
      /* Submultiple window: with a 40 ms anchor this is the only range below
       * 20 ms the scheduler can grant (10 ms = 40/4); for other anchors it
       * still contains a submultiple for every plausible period. */
      return kSubMultipleIntervalMin;
    }
    return interval == Interval::Fast ? fast_interval :
           interval == Interval::Parked ? kParkedInterval : kSlowIntervalMin;
  }

  static constexpr uint16_t interval_max(Interval interval, uint16_t fast_interval,
                                         uint8_t fallback_level) {
    if (interval == Interval::Parked) {
      return kParkedInterval;
    }
    if (interval != Interval::Fast) {
      return kSlowIntervalMax;
    }
    if (fallback_level >= kFastIntervalFallbackLevels) {
      return kFastIntervalFallbackMax;
    }
    if (fallback_level == 2U) {
      return kSubMultipleIntervalMax;
    }
    if (fallback_level == 0U) {
      return fast_interval;
    }
    const uint16_t widened = static_cast<uint16_t>(fast_interval +
                                                   kFastIntervalFallbackSpan);
    return widened < kFastIntervalFallbackMax ? widened : kFastIntervalFallbackMax;
  }

  static constexpr uint16_t max_ce_length(Interval interval,
                                          uint16_t fast_interval,
                                          uint8_t fallback_level) {
    return interval == Interval::Fast && fast_interval == kBulkInterval &&
           fallback_level == 0U ? kBulkMaxCeLength : 0U;
  }

  constexpr Request make_request(uint8_t link, Entry &entry) {
    Request request{};
    request.link = link;
    request.handle = entry.telemetry.handle;
    request.generation = entry.telemetry.generation;
    if (entry.telemetry.state == State::NeedDle) {
      request.procedure = Procedure::Dle;
      request.dle_octets = kRequestedDleOctets;
      request.dle_time_us = kRequestedDleTimeUs;
    } else if (entry.telemetry.state == State::NeedPhy) {
      request.procedure = Procedure::Phy;
      request.tx_phy = kRequestedPhy;
      request.rx_phy = kRequestedPhy;
    } else {
      request.procedure = Procedure::Interval;
      request.interval = entry.interval;
      request.interval_min = interval_min(entry.interval, entry.fast_interval,
                                          entry.interval_fallback_level);
      request.interval_max = interval_max(entry.interval, entry.fast_interval,
                                          entry.interval_fallback_level);
      request.min_ce_length = entry.interval == Interval::Fast &&
                              entry.fast_interval == kBulkInterval ?
                              kBulkMinCeLength : 0U;
      request.max_ce_length = max_ce_length(entry.interval, entry.fast_interval,
                                            entry.interval_fallback_level);
      entry.telemetry.requested_interval_min = request.interval_min;
      entry.telemetry.requested_interval_max = request.interval_max;
      entry.telemetry.requested_min_ce_length = request.min_ce_length;
      entry.telemetry.requested_max_ce_length = request.max_ce_length;
    }
    return request;
  }

  constexpr bool matches_active(const Request &request) const {
    return request.valid() && active_.valid() && request.link == active_.link &&
           request.handle == active_.handle && request.generation == active_.generation &&
           request.procedure == active_.procedure;
  }

  constexpr bool matches_completion(uint16_t handle, uint32_t generation,
                                    Procedure procedure) const {
    return active_accepted_ && active_.procedure == procedure && active_.handle == handle &&
           active_.generation == generation;
  }

  constexpr bool on_completion_status(uint16_t handle, uint32_t generation,
                                      Procedure procedure, uint8_t status, uint32_t now_ms) {
    if (!matches_completion(handle, generation, procedure)) {
      return false;
    }
    return on_request_status(active_, status, now_ms);
  }

  constexpr void complete_active() {
    active_ = Request{};
    active_accepted_ = false;
  }

  std::array<Entry, kLinkCount> links_{};
  Request active_{};
  bool active_accepted_ = false;
};

/* Central-owned upload intent outlives any one BLE connection. The LL tuning
 * model remains per-link/per-generation; this small seam records which single
 * source should be rebound when that Node reconnects. */
class TransferLinkRearmState {
 public:
  constexpr bool begin_source(uint8_t node_id, uint8_t fast_interval) {
    if (!valid_node(node_id)) {
      return false;
    }
    active_node_id_ = node_id;
    fast_interval_ = RecordTransferTuningWire::sanitize_fast_interval(fast_interval);
    armed_generation_ = 0U;
    armed_ = false;
    return true;
  }

  constexpr bool end_source(uint8_t node_id) {
    if (node_id != active_node_id_) {
      return false;
    }
    active_node_id_ = 0U;
    armed_generation_ = 0U;
    armed_ = false;
    return true;
  }

  constexpr void on_link_disconnected(uint8_t node_id) {
    if (node_id == active_node_id_) {
      armed_generation_ = 0U;
      armed_ = false;
    }
  }

  constexpr bool on_link_connected(uint8_t node_id, uint32_t generation) {
    if (node_id != active_node_id_) {
      return false;
    }
    armed_generation_ = generation;
    armed_ = true;
    return true;
  }

  constexpr bool preparation_resolved(uint8_t node_id, uint32_t generation,
                                      bool link_tune_resolved) const {
    return armed_ && node_id == active_node_id_ &&
           generation == armed_generation_ && link_tune_resolved;
  }

  constexpr uint8_t active_node_id() const { return active_node_id_; }
  constexpr uint8_t fast_interval() const { return fast_interval_; }

 private:
  static constexpr bool valid_node(uint8_t node_id) {
    return node_id >= 1U && node_id <= LinkTuneState::kLinkCount;
  }

  uint32_t armed_generation_ = 0U;
  uint8_t active_node_id_ = 0U;
  uint8_t fast_interval_ = LinkTuneState::kFastIntervalMin;
  bool armed_ = false;
};

}  // namespace exo

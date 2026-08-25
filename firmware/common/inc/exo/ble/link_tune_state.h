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
  static constexpr uint32_t kRetryDelayMs = 50U;
  static constexpr uint32_t kCommissioningStartDelayMs = 600U;
  static constexpr uint32_t kProcedureTimeoutMs = 1500U;
  static constexpr uint8_t kMaxTransientAttempts = 3U;
  static constexpr uint8_t kStatusSuccess = 0x00U;
  static constexpr uint8_t kStatusCommandDisallowed = 0x0CU;
  static constexpr uint8_t kStatusControllerBusy = 0x3AU;
  static constexpr uint8_t kStatusInvalidParameters = 0x12U;
  static constexpr uint8_t kStatusNotRequested = 0xFFU;
  static constexpr uint8_t kStatusTimeout = 0xFEU;

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
    return true;
  }

  constexpr Request issue_next(uint32_t now_ms) {
    if (active_.valid()) {
      return Request{};
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
    Entry &entry = links_[request.link];
    entry.telemetry.status = status;
    active_ = Request{};
    active_accepted_ = false;
    if (is_transient(status)) {
      ++entry.telemetry.retries;
      if (entry.telemetry.retries >= kMaxTransientAttempts) {
        entry.telemetry.state = State::Degraded;
        entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
      } else {
        entry.telemetry.state = need_state(request.procedure);
        entry.retry_at_ms = now_ms + kRetryDelayMs;
      }
    } else {
      entry.telemetry.state = State::Failed;
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
    if (entry.fast_after_commission) {
      entry.interval = Interval::Fast;
      entry.telemetry.state = State::NeedInterval;
    } else {
      entry.telemetry.state = State::Ready;
    }
    entry.fast_after_commission = false;
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
    entry.telemetry.confirmed_interval = interval;
    entry.telemetry.status = kStatusSuccess;
    entry.telemetry.state = State::Ready;
    entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
    complete_active();
    return true;
  }

  constexpr bool on_timeout(uint32_t now_ms) {
    if (!active_.valid() || !active_accepted_ || now_ms < links_[active_.link].deadline_ms) {
      return false;
    }
    Entry &entry = links_[active_.link];
    entry.telemetry.status = kStatusTimeout;
    entry.telemetry.state = State::Degraded;
    entry.telemetry.preparation_duration_ms = now_ms - entry.connected_at_ms;
    complete_active();
    return true;
  }

  constexpr bool begin_slow_restore(uint8_t link, uint32_t generation) {
    if (link >= kLinkCount) {
      return false;
    }
    Entry &entry = links_[link];
    if (!entry.connected || entry.telemetry.generation != generation ||
        (active_.valid() && active_.link == link)) {
      return false;
    }
    if (entry.telemetry.state == State::NeedDle || entry.telemetry.state == State::WaitDle ||
        entry.telemetry.state == State::NeedPhy || entry.telemetry.state == State::WaitPhy) {
      entry.fast_after_commission = true;
      return true;
    }
    if (entry.telemetry.state != State::Ready && entry.telemetry.state != State::Degraded) {
      return false;
    }
    entry.interval = Interval::Slow;
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
    if (!entry.connected || entry.telemetry.generation != generation ||
        (active_.valid() && active_.link == link) ||
        (entry.telemetry.state != State::Ready && entry.telemetry.state != State::Degraded)) {
      return false;
    }
    entry.fast_interval = RecordTransferTuningWire::sanitize_fast_interval(fast_interval);
    entry.interval = Interval::Fast;
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

  constexpr Request active_request() const { return active_; }

 private:
  struct Entry {
    Telemetry telemetry{};
    uint32_t next_generation = 1U;
    uint32_t connected_at_ms = 0U;
    uint32_t retry_at_ms = 0U;
    uint32_t deadline_ms = 0U;
    Interval interval = Interval::Fast;
    uint16_t fast_interval = kFastIntervalMin;
    bool fast_after_commission = false;
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

  static constexpr uint16_t interval_min(Interval interval, uint16_t fast_interval) {
    return interval == Interval::Fast ? fast_interval : kSlowIntervalMin;
  }

  static constexpr uint16_t interval_max(Interval interval, uint16_t fast_interval) {
    return interval == Interval::Fast ? fast_interval : kSlowIntervalMax;
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
      request.interval_min = interval_min(entry.interval, entry.fast_interval);
      request.interval_max = interval_max(entry.interval, entry.fast_interval);
      entry.telemetry.requested_interval_min = request.interval_min;
      entry.telemetry.requested_interval_max = request.interval_max;
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

}  // namespace exo

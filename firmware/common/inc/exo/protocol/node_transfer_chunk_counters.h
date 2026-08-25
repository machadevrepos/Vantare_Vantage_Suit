#pragma once

#include <cstdint>

namespace exo {

/* Per-source receive evidence for raw-relay-off throughput benchmarks.
 * Unique counts advance only after the coordinator commits a new transfer
 * window position. Duplicate counts advance only when the transfer window
 * classifies an already accepted chunk. */
class NodeTransferChunkCounters {
 public:
  constexpr void reset_session() {
    source_id_ = 0U;
    unique_accepted_ = 0U;
    duplicates_ = 0U;
  }

  constexpr void begin_source(uint8_t source_id) {
    source_id_ = source_id >= 1U && source_id <= 4U ? source_id : 0U;
    unique_accepted_ = 0U;
    duplicates_ = 0U;
  }

  constexpr void note_unique_accepted() {
    if (source_id_ != 0U && unique_accepted_ != 0xFFFFFFFFUL) {
      ++unique_accepted_;
    }
  }

  constexpr void note_duplicate() {
    if (source_id_ != 0U && duplicates_ != 0xFFFFFFFFUL) {
      ++duplicates_;
    }
  }

  constexpr uint8_t source_id() const { return source_id_; }
  constexpr uint32_t unique_accepted() const { return unique_accepted_; }
  constexpr uint32_t duplicates() const { return duplicates_; }

 private:
  uint8_t source_id_ = 0U;
  uint32_t unique_accepted_ = 0U;
  uint32_t duplicates_ = 0U;
};

}  // namespace exo

#pragma once

#include <cstdint>

namespace exo {

/* Per-source receive evidence for raw-relay-off throughput benchmarks.
 * Unique counts advance only after the coordinator commits a new transfer
 * window position. Retransmitted counts advance from validated wire evidence,
 * independently of whether the frame repairs a hole or is a duplicate. */
class NodeTransferChunkCounters {
 public:
  constexpr void reset_session() {
    source_id_ = 0U;
    unique_accepted_ = 0U;
    retransmitted_ = 0U;
  }

  constexpr void begin_source(uint8_t source_id) {
    source_id_ = source_id >= 1U && source_id <= 4U ? source_id : 0U;
    unique_accepted_ = 0U;
    retransmitted_ = 0U;
  }

  constexpr void note_unique_accepted() {
    if (source_id_ != 0U && unique_accepted_ != 0xFFFFFFFFUL) {
      ++unique_accepted_;
    }
  }

  constexpr void note_retransmitted() {
    if (source_id_ != 0U && retransmitted_ != 0xFFFFFFFFUL) {
      ++retransmitted_;
    }
  }

  constexpr uint8_t source_id() const { return source_id_; }
  constexpr uint32_t unique_accepted() const { return unique_accepted_; }
  constexpr uint32_t retransmitted() const { return retransmitted_; }

 private:
  uint8_t source_id_ = 0U;
  uint32_t unique_accepted_ = 0U;
  uint32_t retransmitted_ = 0U;
};

}  // namespace exo

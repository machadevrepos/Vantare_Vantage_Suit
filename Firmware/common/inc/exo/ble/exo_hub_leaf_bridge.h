#ifndef EXO_HUB_LEAF_BRIDGE_H_
#define EXO_HUB_LEAF_BRIDGE_H_

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

uint8_t exo_hub_leaf_stream_ingest(uint8_t node_id,
                                   uint8_t sensor_id,
                                   const uint8_t *payload,
                                   uint8_t payload_len);
uint8_t exo_hub_leaf_record_done_ingest(const uint8_t *payload, uint16_t length);
void exo_hub_leaf_record_frame_ingest(uint8_t node_id,
                                      const uint8_t *payload,
                                      uint16_t payload_len);
void exo_hub_leaf_topology_touch(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif

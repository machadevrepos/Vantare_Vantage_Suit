#ifndef RECORDING_BRIDGE_H_
#define RECORDING_BRIDGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t exo_node_ble_write(const uint8_t *payload, uint8_t length);
uint8_t exo_hub_ble_write(const uint8_t *payload, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif

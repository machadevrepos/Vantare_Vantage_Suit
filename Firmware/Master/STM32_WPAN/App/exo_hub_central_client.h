#ifndef EXO_HUB_CENTRAL_CLIENT_H_
#define EXO_HUB_CENTRAL_CLIENT_H_

#include <stdint.h>
#include "blepipe_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

void exo_ble_debug_printf(const char *fmt, ...);

void exo_hub_central_client_init(void);
void exo_hub_central_client_set_ble_ready(void);
void exo_hub_central_client_process(void);
void exo_hub_central_client_request_scan(void);

uint8_t exo_hub_central_client_broadcast_blepipe(uint8_t msg_type,
                                                 uint16_t src_id,
                                                 const uint8_t *payload,
                                                 uint16_t payload_len);
uint8_t exo_hub_central_client_broadcast_blepipe_mask(uint8_t msg_type,
                                                      uint16_t src_id,
                                                      const uint8_t *payload,
                                                      uint16_t payload_len);
uint8_t exo_hub_central_client_send_blepipe_to_node(uint8_t node_id,
                                                    uint8_t msg_type,
                                                    uint16_t src_id,
                                                    const uint8_t *payload,
                                                    uint16_t payload_len);
uint8_t exo_hub_central_client_send_raw_to_node(uint8_t node_id,
                                                const uint8_t *payload,
                                                uint16_t payload_len);
uint8_t exo_hub_central_client_ready_node_mask(void);
uint32_t exo_hub_central_client_maximum_duration_ms(uint8_t node_mask);
uint8_t exo_hub_central_client_ready_node_count(void);
uint8_t exo_hub_central_client_transport_ready_node_mask(void);
uint8_t exo_hub_central_client_transport_ready_node_count(void);

void exo_hub_central_client_on_connection_complete(uint8_t initiated_as_client,
                                                   uint8_t status,
                                                   uint16_t connection_handle,
                                                   uint8_t peer_address_type,
                                                   const uint8_t *peer_address);
void exo_hub_central_client_on_disconnection_complete(uint16_t connection_handle,
                                                      uint8_t reason);

#ifdef __cplusplus
}
#endif

#endif

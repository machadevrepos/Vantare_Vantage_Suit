#ifndef EXO_HUB_CENTRAL_CLIENT_H_
#define EXO_HUB_CENTRAL_CLIENT_H_

#include <stdint.h>
#include <exo/protocol/blepipe_proto.h>

#ifdef __cplusplus
extern "C" {
#endif

void exo_ble_debug_printf(const char *fmt, ...);

void exo_hub_central_client_init(void);
void exo_hub_central_client_set_ble_ready(void);
void exo_hub_central_client_process(void);
void exo_hub_central_client_request_scan(void);
void exo_hub_central_client_set_discovery_hold(uint8_t hold);
/* Arm one direct-address re-connect of a dropped Node while discovery is
 * held for an active session; ignored outside the hold. */
void exo_hub_central_client_request_targeted_reconnect(uint8_t node_id);
/* Session-upload link timing: fast=1 requests the supplied sanitized interval
 * through the global completion-driven arbiter; fast=0 restores the 30-50 ms
 * multi-link timing. Request queuing/acceptance is never confirmation. */
void exo_hub_central_client_set_transfer_timing(uint8_t node_id, uint8_t fast,
                                                uint8_t fast_interval);
/* True only when the exact connected generation armed by the most recent fast
 * request for this Node reached Ready or an explicit Degraded/Failed fallback. */
uint8_t exo_hub_central_client_transfer_preparation_resolved(uint8_t node_id);

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
void exo_hub_central_client_on_data_length_change(uint16_t connection_handle,
                                                  uint16_t max_tx_octets,
                                                  uint16_t max_tx_time,
                                                  uint16_t max_rx_octets,
                                                  uint16_t max_rx_time);
void exo_hub_central_client_on_phy_update_complete(uint8_t status,
                                                   uint16_t connection_handle,
                                                   uint8_t tx_phy,
                                                   uint8_t rx_phy);
void exo_hub_central_client_on_connection_update_complete(uint8_t status,
                                                          uint16_t connection_handle,
                                                          uint16_t conn_interval,
                                                          uint16_t conn_latency,
                                                          uint16_t supervision_timeout);

#ifdef __cplusplus
}
#endif

#endif

#ifndef BLE_SESSION_CONTROL_H_
#define BLE_SESSION_CONTROL_H_

#include <stdint.h>

namespace exo {

static constexpr uint8_t kSessionMasterSourceBit = 0x01U;
static constexpr uint8_t kSessionNodeMask = 0x1EU;

constexpr bool session_node_mask_valid(uint8_t selected_node_mask)
{
    return selected_node_mask != 0U &&
           (selected_node_mask & static_cast<uint8_t>(~kSessionNodeMask)) == 0U;
}

constexpr uint8_t session_missing_node_mask(uint8_t selected_node_mask,
                                           uint8_t ready_node_mask)
{
    return static_cast<uint8_t>(selected_node_mask &
                                static_cast<uint8_t>(~ready_node_mask));
}

constexpr uint8_t session_expected_source_mask(uint8_t selected_node_mask)
{
    return static_cast<uint8_t>(kSessionMasterSourceBit |
                                (selected_node_mask & kSessionNodeMask));
}

} // namespace exo

#endif

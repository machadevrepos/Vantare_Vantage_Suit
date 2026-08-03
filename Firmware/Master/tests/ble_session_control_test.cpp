#include <stdint.h>

#include "../../LIBRARY/CUSTOM/BLE_RECORD_PROTOCOL.h"
#include "../../LIBRARY/CUSTOM/BLE_SESSION_CONTROL.h"

static_assert(sizeof(exo::StopRecordMessage) == 5U,
              "StopRecord wire format must remain compact");
static_assert(sizeof(exo::StartSessionMessage) == 19U,
              "StartSession wire format must match the browser encoder");

static_assert(exo::session_node_mask_valid(0x02U),
              "A selected one-node session must be valid");
static_assert(exo::session_node_mask_valid(0x1EU),
              "NODE1 through NODE4 must be valid");
static_assert(!exo::session_node_mask_valid(0x00U),
              "At least one Node must be selected");
static_assert(!exo::session_node_mask_valid(0x20U),
              "Bits outside NODE1 through NODE4 must be rejected");
static_assert(exo::session_missing_node_mask(0x0AU, 0x02U) == 0x08U,
              "Missing selected participants must be reported exactly");
static_assert(exo::session_expected_source_mask(0x0AU) == 0x0BU,
              "The Master bit must be added to selected Node bits");

int main()
{
    return 0;
}

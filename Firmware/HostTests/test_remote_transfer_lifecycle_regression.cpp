#include <cstdint>
#include <iostream>
#include <type_traits>

#include "HUB_LEAF_BLE_MANAGER.h"

namespace {
int failures = 0;
#define EXPECT_TRUE(expr) do { \
    if (!(expr)) { \
        std::cerr << __FILE__ << ':' << __LINE__ << ": expected " #expr "\n"; \
        ++failures; \
    } \
} while (0)

exo::RecordDoneMessage make_done(uint8_t node_id, uint32_t session_id, uint32_t crc)
{
    exo::RecordDoneMessage done{};
    done.command = exo::RecordCommand::RecordDone;
    done.node_id = node_id;
    done.session_id = session_id;
    done.total_size = sizeof(exo::SessionHeader) + 64U;
    done.payload_crc32 = crc;
    return done;
}
}  // namespace

int main()
{
    using Manager = exo::ble_hub::HubLeafBleManager;
    static_assert(std::is_same_v<
        decltype(std::declval<Manager &>().on_ble_reliable_pause(uint32_t{}, uint16_t{})),
        bool>, "Pause must report whether the source/session owns the transfer");

    Manager manager;
    const auto first = make_done(1U, 77U, 0x11111111U);
    const auto second = make_done(2U, 77U, 0x22222222U);
    EXPECT_TRUE(manager.queue_record_done(first));
    EXPECT_TRUE(manager.queue_record_done(second));

    exo::RecordDoneMessage selected{};
    EXPECT_TRUE(manager.pop_next_record_done(selected));
    EXPECT_TRUE(selected.node_id == 1U);
    EXPECT_TRUE(manager.active_source_id() == 1U);
    EXPECT_TRUE(manager.active_session_id() == 77U);

    EXPECT_TRUE(!manager.on_ble_reliable_pause(77U, 2U));
    EXPECT_TRUE(!manager.paused());
    EXPECT_TRUE(manager.on_ble_reliable_pause(77U, 1U));
    EXPECT_TRUE(manager.paused());

    EXPECT_TRUE(!manager.on_ble_reliable_resume(77U, 2U));
    EXPECT_TRUE(manager.paused());
    EXPECT_TRUE(manager.on_ble_reliable_resume(77U, 1U));
    EXPECT_TRUE(!manager.paused());

    EXPECT_TRUE(!manager.on_ble_reliable_cancel(77U, 2U));
    EXPECT_TRUE(manager.active_source_id() == 1U);
    EXPECT_TRUE(manager.active_session_id() == 77U);
    EXPECT_TRUE(manager.on_ble_reliable_cancel(77U, 1U));
    EXPECT_TRUE(manager.active_source_id() == 0U);
    EXPECT_TRUE(manager.active_session_id() == 0U);

    EXPECT_TRUE(manager.pop_next_record_done(selected));
    EXPECT_TRUE(selected.node_id == 2U);

    if (failures != 0) {
        std::cerr << failures << " remote lifecycle regression check(s) failed\n";
        return 1;
    }
    std::cout << "remote transfer lifecycle regression checks passed\n";
    return 0;
}

#!/usr/bin/env python3
"""Source-level guards for the Master BLE commissioning/desktop path."""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def require(ok: bool, message: str, failures: list[str]) -> None:
    if not ok:
        failures.append(message)


def function_body(text: str, signature: str, next_signature: str | None) -> str:
    start = text.find(signature)
    if start < 0:
        return ""
    end = len(text) if next_signature is None else text.find(next_signature, start + len(signature))
    if end < 0:
        return ""
    return text[start:end]


def main() -> int:
    failures: list[str] = []
    central = (ROOT / "Firmware/Master/STM32_WPAN/App/exo_hub_central_client.c").read_text(encoding="utf-8")
    master_app_ble = (ROOT / "Firmware/Master/STM32_WPAN/App/app_ble.c").read_text(encoding="utf-8")
    master_conf = (ROOT / "Firmware/Master/Core/Inc/app_conf.h").read_text(encoding="utf-8")

    leaf_max = re.search(r"#define\s+EXO_HUB_LEAF_MAX\s+(\d+)U", central)
    cfg_links = re.search(r"#define\s+CFG_BLE_NUM_LINK\s+(\d+)", master_conf)
    require(leaf_max is not None, "Master central client must declare EXO_HUB_LEAF_MAX", failures)
    require(cfg_links is not None, "Master BLE config must declare CFG_BLE_NUM_LINK", failures)
    if leaf_max is not None and cfg_links is not None:
        leaf_count = int(leaf_max.group(1))
        link_count = int(cfg_links.group(1))
        require(leaf_count == 4,
                "Master commissioning target must be the four-node suit topology", failures)
        require(link_count >= leaf_count + 1,
                "Master BLE link budget must leave one connection for Exoskeleton.html", failures)

    require("g_leaf_scan_holds_advertising = 1U;" in master_app_ble and
            "Adv_Request(APP_BLE_FAST_ADV);" in master_app_ble,
            "Leaf scans must explicitly resume desktop advertising when they idle", failures)

    gap_body = function_body(
        central,
        "void aci_gap_proc_complete_event(",
        "void aci_att_read_by_group_type_resp_event(",
    )
    require(gap_body != "", "Master central GAP completion handler must be locatable", failures)
    if gap_body:
        queue_pos = gap_body.find("const uint8_t next_slot = exo_find_next_connectable_slot();")
        hold_pos = gap_body.find("g_discovery_hold != 0U")
        idle_pos = gap_body.find("APP_BLE_LeafClientScanIdle();")
        require(hold_pos >= 0 and idle_pos > hold_pos and
                (queue_pos < 0 or hold_pos < queue_pos),
                "Discovery hold must idle scans and resume desktop advertising before queuing a node connection", failures)

    hold_body = function_body(
        central,
        "void exo_hub_central_client_set_discovery_hold(",
        None,
    )
    require(hold_body != "", "Master discovery-hold setter must be locatable", failures)
    if hold_body:
        require("g_scan_requested = 0U;" in hold_body and
                "APP_BLE_LeafClientScanIdle();" in hold_body,
                "Entering discovery hold must leave the desktop advertising path recoverable", failures)

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1

    print("BLE commissioning path invariants passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

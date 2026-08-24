#!/usr/bin/env python3
"""Source-level guards for reliable StopRecord delivery from Master to nodes."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")
def main() -> int:
    failures=[]
    master=read("Firmware/Master/Core/Src/main.cpp")
    node=read("Firmware/Node/Core/Src/main.cpp")
    def require(ok,msg):
        if not ok: failures.append(msg)
    a=master.find("static bool stop_active_session")
    b=master.find("static void record_sync_process",a)
    block=master[a:b] if a>=0 and b>=0 else ""
    require("stop_stream_command" not in block,"StopRecord must not be preceded by a separate stream-stop write")
    require("record_stop_sync_begin(message, g_active_session_node_mask)" in block,"StopRecord must enter reliable retry state")
    require("static void record_stop_sync_process()" in master,"Master must implement StopRecord retry service")
    require("record_stop_sync_process();" in master,"Master superloop must service StopRecord retries")
    a=master.find('extern "C" void exo_hub_leaf_control_ingest')
    b=master.find('extern "C" void exo_hub_leaf_topology_touch',a)
    ctrl=master[a:b] if a>=0 and b>=0 else ""
    stop_ack=ctrl.find("RecordCommand::StopRecord")
    start_guard=ctrl.find("!g_record_sync.active")
    require(stop_ack>=0 and start_guard>=0 and stop_ack<start_guard,"Stop ACK handling must occur before start-sync guard")
    require("g_record_stop_sync.ack_mask" in ctrl,"Accepted Stop ACKs must be tracked per node")
    a=node.find("case static_cast<uint8_t>(exo::RecordCommand::StopRecord):")
    b=node.find("case static_cast<uint8_t>(exo::RecordCommand::SessionCompleteAck):",a)
    n=node[a:b] if a>=0 and b>=0 else ""
    require("[BLE][NODE][STOP]" in n,"Node must log when StopRecord is processed")
    if failures:
        for f in failures: print("ERROR:",f,file=sys.stderr)
        return 1
    print("stop delivery invariant guards passed")
    return 0
if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path

ROOT = Path.cwd()


def replace_once(relative: str, old: str, new: str) -> None:
    path = ROOT / relative
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{relative}: expected one match, found {count}")
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h",
    "#include <BLE_RECORD_PROTOCOL.h>\n",
    "#include <BLE_RECORD_PROTOCOL.h>\n#include <RECORDING_TYPES.h>\n",
)
replace_once(
    "Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h",
    "  bool queue_record_done(const exo::RecordDoneMessage &message) {\n"
    "    if (message.node_id < 1U || message.node_id > kMaxLeaves) return false;\n",
    "  bool queue_record_done(const exo::RecordDoneMessage &message) {\n"
    "    if (message.command != exo::RecordCommand::RecordDone ||\n"
    "        message.node_id < 1U || message.node_id > kMaxLeaves ||\n"
    "        message.session_id == 0U ||\n"
    "        message.total_size < sizeof(exo::SessionHeader)) {\n"
    "      return false;\n"
    "    }\n",
)
replace_once(
    "Firmware/Master/Core/Inc/HUB_LEAF_BLE_MANAGER.h",
    "  static constexpr uint32_t kNormalPreviewIntervalMs = 40U;\n"
    "  static constexpr uint32_t kCongestedPreviewIntervalMs = 80U;\n",
    "  // Aggregate round-robin cadence. With four ready Nodes this yields\n"
    "  // 40 ms per source normally and 80 ms per source under backpressure.\n"
    "  static constexpr uint32_t kNormalPreviewIntervalMs = 10U;\n"
    "  static constexpr uint32_t kCongestedPreviewIntervalMs = 20U;\n",
)

replace_once(
    "Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h",
    "        if ((completed_source_mask_ & source_bit) != 0U) {\n"
    "            return set_nonterminal(training_csv::TrainingCsvLogOperation::DuplicateSource,\n"
    "                    FR_INVALID_PARAMETER);\n"
    "        }\n"
    "        if (!flush_buffer() || !synchronize(now_ms)) return false;\n",
    "        if ((completed_source_mask_ & source_bit) != 0U) {\n"
    "            return set_nonterminal(training_csv::TrainingCsvLogOperation::DuplicateSource,\n"
    "                    FR_INVALID_PARAMETER);\n"
    "        }\n"
    "        if (!metadata_valid_[source_id][kBnoSensorIndex] ||\n"
    "                !metadata_valid_[source_id][kIcmSensorIndex]) {\n"
    "            return set_nonterminal(\n"
    "                    training_csv::TrainingCsvLogOperation::InvalidSourceMetadata,\n"
    "                    FR_INVALID_PARAMETER);\n"
    "        }\n"
    "        if (!flush_buffer() || !synchronize(now_ms)) return false;\n",
)
replace_once(
    "Firmware/Master/Core/Inc/MASTER_TRAINING_CSV_LOGGER.h",
    "        result = ops_->close_fn(&marker);\n"
    "        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, result);\n"
    "        published_ = true;\n"
    "        return true;\n",
    "        result = ops_->close_fn(&marker);\n"
    "        if (result != FR_OK) return set_terminal(training_csv::TrainingCsvLogOperation::MarkerClose, result);\n"
    "        terminal_error_ = false;\n"
    "        last_operation_ = training_csv::TrainingCsvLogOperation::None;\n"
    "        last_result_ = FR_OK;\n"
    "        published_ = true;\n"
    "        return true;\n",
)

replace_once(
    "Firmware/Master/Core/Src/main.c",
    "\texo::RecordDoneMessage message { };\n"
    "\tmemcpy(&message, payload, sizeof(message));\n"
    "\tif (message.command == exo::RecordCommand::RecordDone &&\n"
    "\t\t\tmessage.node_id >= 1U && message.node_id <= 4U &&\n"
    "\t\t\tmessage.total_size >= sizeof(exo::SessionHeader)) {\n"
    "\t\tconst uint8_t training_index = static_cast<uint8_t>(message.node_id - 1U);\n"
    "\t\tg_training_node_done[training_index] = message;\n"
    "\t\tg_training_node_done_valid[training_index] = true;\n"
    "\t}\n"
    "\tmaster_training_csv_coordinator.on_node_record_done(message);\n",
    "\texo::RecordDoneMessage message { };\n"
    "\tmemcpy(&message, payload, sizeof(message));\n"
    "\tif (message.command != exo::RecordCommand::RecordDone ||\n"
    "\t\t\tmessage.node_id < 1U || message.node_id > 4U ||\n"
    "\t\t\tmessage.session_id == 0U ||\n"
    "\t\t\tmessage.total_size < sizeof(exo::SessionHeader)) {\n"
    "\t\treturn 0U;\n"
    "\t}\n"
    "\tconst uint8_t training_index = static_cast<uint8_t>(message.node_id - 1U);\n"
    "\tg_training_node_done[training_index] = message;\n"
    "\tg_training_node_done_valid[training_index] = true;\n"
    "\tmaster_training_csv_coordinator.on_node_record_done(message);\n",
)

replace_once(
    "Docs/Superpowers/Specs/2026-08-03-four-node-live-preview-training-csv-design.md",
    "- `timestamp_quality_flags`\n\n### BNO85 raw and calibrated columns\n",
    "- `timestamp_quality_flags`\n\n"
    "`timestamp_quality_flags` is a bitmask. Bit `0x00000001` means the current "
    "timestamp repeated or decreased relative to the previous accepted sample for "
    "that source and sensor. Additional quality conditions use distinct bits; "
    "producers combine simultaneous conditions with bitwise OR and consumers must "
    "test individual bits rather than compare the full field to one value.\n\n"
    "### BNO85 raw and calibrated columns\n",
)
replace_once(
    "Firmware/Project Details.md",
    "The Master uses a shared preview pacing gate: 40 ms under normal conditions and 80 ms after BLE backpressure. A failed latest-value sample is retained, other ready sources remain eligible, and normal cadence resumes after a stable recovery period.\n",
    "The Master uses a shared round-robin pacing gate at 10 ms normally and 20 ms after BLE backpressure. With four ready Nodes, this preserves 40 ms and 80 ms per-source cadence respectively. A failed latest-value sample is retained, other ready sources remain eligible, and normal cadence resumes after a stable recovery period.\n",
)

replace_once(
    "Firmware/HostTests/test_ble_only_cleanup.py",
    "    if \"node_stream_enabled\" not in node_main:\n"
    "        failures.append(\"Node main.c does not expose direct BLE stream state\")\n\n"
    "    if failures:\n",
    "    if \"node_stream_enabled\" not in node_main:\n"
    "        failures.append(\"Node main.c does not expose direct BLE stream state\")\n\n"
    "    record_done_guards = (\n"
    "        \"message.command != exo::RecordCommand::RecordDone\",\n"
    "        \"message.session_id == 0U\",\n"
    "        \"message.total_size < sizeof(exo::SessionHeader)\",\n"
    "    )\n"
    "    for guard in record_done_guards:\n"
    "        if guard not in master_main:\n"
    "            failures.append(f\"Master record-done ingest lacks guard: {guard}\")\n\n"
    "    if failures:\n",
)

print("final CodeRabbit fixes applied")

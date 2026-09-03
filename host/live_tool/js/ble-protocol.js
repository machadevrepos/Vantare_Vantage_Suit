/**
 * BLE transport for the Vantare live inference tool.
 *
 * Wire contracts verified against:
 * - firmware/common/inc/exo/protocol/ble_stream_v2.h      (B1 envelope, 14-byte header)
 * - firmware/common/inc/exo/types/recording_types.h       (Bno85SampleV3 56 B, Icm45686SampleV4 20 B)
 * - firmware/common/inc/exo/protocol/blepipe_proto.h      (20-byte control header + CRC16-CCITT)
 * - firmware/Master/Core/Src/main.cpp                     (0xA0/0xA1/0xA2/0xA3 command handlers)
 * - host/desktop_tool/Exoskeleton.html                    (UUIDs, blepipe encoding, decoders)
 *
 * ICM raw-to-physical scale factors must match the converter that produced the
 * training data (host/desktop_tool/vantage_bin_to_csv.py):
 *   accel_g = raw * 4.0 / 32768.0
 *   gyro_dps = raw * 2000.0 / 32768.0
 */

/**
 * The Master exposes ONE 128-bit service, BLEPipe `3f881000`, with every
 * characteristic inside it (firmware/Master/Core/Src/ble/custom_stm.cpp):
 *
 *   3f881001  PIPEDATATX     sensor data     notify
 *   3f881002  PIPECONTROLRX  commands        write
 *   3f881003  PIPECONTROLTX  command acks    notify
 *   3f881004  PIPESTATUSTX   status/recovery notify
 *   3f881005  PIPECONFIGRW   config          read/write
 *
 * There is no separate control or status service. Earlier drafts carried
 * `3f882000`/`3f883000` entries copied from the desktop tool, where they are
 * vestigial and never resolved; resolving them here failed with
 * "No Services matching UUID ... found in Device" and silently cost status
 * notifications. Fetch every characteristic from `serviceUuid`.
 */
export const BLE_CFG = {
  deviceNamePrefix: "HUB0001",
  deviceNamePrefixFallback: "HUB",
  serviceUuid: "3f881000-b4a5-4f7c-9b60-98e0b5c8a000",
  imuCharUuid: "3f881001-b4a5-4f7c-9b60-98e0b5c8a000",
  cmdCharUuid: "3f881002-b4a5-4f7c-9b60-98e0b5c8a000",
  cmdAckCharUuid: "3f881003-b4a5-4f7c-9b60-98e0b5c8a000",
  statusCharUuid: "3f881004-b4a5-4f7c-9b60-98e0b5c8a000",
  configCharUuid: "3f881005-b4a5-4f7c-9b60-98e0b5c8a000",
  maxPayloadLen: 244,
};

export const CMD = {
  START_STREAM: 0xa0,
  STOP_STREAM: 0xa1,
  SET_INTERVAL: 0xa2,
  SET_ERM: 0xa3,
  SET_BUZZER: 0xa4,
  SET_RGB: 0xa5,
  TOUCH_TEST: 0xa6,
  HAPTIC_PULSE: 0xa7,
  /** Master own-IMU preview toggle (payload[1] 0=off 1=on). Live inference
   * turns it off: the model consumes N2-N4 only, and the ~2.6 KB/s of
   * master-own frames oversubscribes the ~9.8 KB/s link. */
  MASTER_OWN_STREAM: 0xa8,
  QUERY_NODE_ID: 0xb1,
  REDISCOVER: 0xb2,
  RESET_RETAINED: 0xb3,
  DISCOVERED_NODES: 0xb4,
};

/** command byte -> readable name, for logging Master ACK/NACK frames. */
export const CMD_NAME = Object.fromEntries(
  Object.entries(CMD).map(([name, code]) => [code, name])
);

/** Actuator override bit understood by 0xA5 on both Master and Node. */
export const RGB_OVERRIDE_OFF = 0x80;

/** Section 6.4 bounds, enforced by the Node and mirrored here. */
export const PULSE_BOUNDS = {
  minIntensityPercent: 1,
  maxIntensityPercent: 100,
  minDurationMs: 50,
  maxDurationMs: 500,
};

export const BLEPIPE = {
  PROTO_VER: 1,
  HDR_LEN: 20,
  CRC_LEN: 2,
  MSG_COMMAND: 0x10,
  MSG_ACK: 0x12,
  MSG_NACK: 0x13,
  MSG_STREAM_CONTROL: 0x32,
  MSG_TOPOLOGY: 0x21,
  MSG_LOG: 0x24,
  DST_BROADCAST: 0xffff,
  SRC_PHONE: 0x1000,
};

export const SENSOR = {
  BNO: 0x01,
  ICM: 0x02,
};

// Column names match the training pipeline exactly
// (host/notebooks, Vantare_Bicep_Curl_Training_ONNX.ipynb, cell 5).
export const BNO_COLUMNS = [
  "quat_i", "quat_j", "quat_k", "quat_real",
  "linear_accel_x_mps2", "linear_accel_y_mps2", "linear_accel_z_mps2",
  "gravity_x_mps2", "gravity_y_mps2", "gravity_z_mps2",
  "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
];
export const ICM_COLUMNS = [
  "accel_x_g", "accel_y_g", "accel_z_g",
  "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
];

const V2 = {
  FRAME_ID: 0xb1,
  HEADER_SIZE: 14,
  BNO_SAMPLE_SIZE: 56,
  ICM_SAMPLE_SIZE_V4: 20,
  ICM_SAMPLE_SIZE_V3: 12,
};

/** Sources the V1 model consumes. Only these feed preprocessing. */
export const NODE_IDS = [2, 3, 4];

/** Master source id in the B1 envelope (firmware kMasterNodeId). */
export const MASTER_ID = 0;

/**
 * Every source the page will decode and visualize. The Master streams its own
 * BNO/ICM through the same B1 envelope with node_id 0, and Node 1 exists in
 * four-node builds, so both are charted even though the V1 model ignores them.
 * Dropping unknown ids here is what previously made the Master graph dead.
 */
export const DISPLAY_SOURCE_IDS = [0, 1, 2, 3, 4];

export function sourceLabel(sourceId) {
  return sourceId === MASTER_ID ? "MASTER" : `N${sourceId}`;
}

export function crc16Ccitt(bytes) {
  let crc = 0xffff;
  for (const byte of bytes) {
    crc ^= byte << 8;
    for (let bit = 0; bit < 8; bit += 1) {
      crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
    }
  }
  return crc & 0xffff;
}

export function blepipeEncode(msgType, payload, dstId = BLEPIPE.DST_BROADCAST, seq = 1) {
  const body = payload instanceof Uint8Array ? payload : Uint8Array.from(payload || []);
  const out = new Uint8Array(BLEPIPE.HDR_LEN + body.length + BLEPIPE.CRC_LEN);
  const dv = new DataView(out.buffer);
  dv.setUint8(0, BLEPIPE.PROTO_VER);
  dv.setUint8(1, msgType);
  dv.setUint8(2, 0);
  dv.setUint8(3, 0);
  dv.setUint16(4, BLEPIPE.SRC_PHONE, true);
  dv.setUint16(6, dstId, true);
  dv.setUint32(8, seq >>> 0, true);
  dv.setUint32(12, Math.floor(performance.now()) >>> 0, true);
  dv.setUint16(16, body.length, true);
  dv.setUint16(18, 0, true);
  out.set(body, BLEPIPE.HDR_LEN);
  dv.setUint16(BLEPIPE.HDR_LEN + body.length, crc16Ccitt(out.slice(0, BLEPIPE.HDR_LEN + body.length)), true);
  return out;
}

export function blepipeDecode(packet) {
  const data = packet instanceof Uint8Array ? packet : new Uint8Array(packet);
  if (!data || data.length < BLEPIPE.HDR_LEN + BLEPIPE.CRC_LEN) return { ok: false, reason: "short" };
  const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
  if (dv.getUint8(0) !== BLEPIPE.PROTO_VER) return { ok: false, reason: "version" };
  const payloadLen = dv.getUint16(16, true);
  const totalLen = BLEPIPE.HDR_LEN + payloadLen + BLEPIPE.CRC_LEN;
  if (totalLen !== data.length) return { ok: false, reason: "length" };
  const expected = dv.getUint16(BLEPIPE.HDR_LEN + payloadLen, true);
  const actual = crc16Ccitt(data.slice(0, BLEPIPE.HDR_LEN + payloadLen));
  if (expected !== actual) return { ok: false, reason: "crc" };
  return {
    ok: true,
    msgType: dv.getUint8(1),
    srcId: dv.getUint16(4, true),
    dstId: dv.getUint16(6, true),
    payload: data.slice(BLEPIPE.HDR_LEN, BLEPIPE.HDR_LEN + payloadLen),
  };
}

export function parseEnvelopeV2(bytes) {
  if (bytes.length < V2.HEADER_SIZE) return { ok: false, reason: "short" };
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const frameId = dv.getUint8(0);
  const version = dv.getUint8(1);
  if (frameId !== V2.FRAME_ID || version !== 1) return { ok: false, reason: "id/version" };
  const nodeId = dv.getUint16(2, true);
  const sensorId = dv.getUint8(4);
  const sequence = dv.getUint16(5, true);
  const timeMs = dv.getUint32(7, true);
  const payloadLen = dv.getUint16(11, true);
  if (V2.HEADER_SIZE + payloadLen > bytes.length) return { ok: false, reason: "len" };
  return {
    ok: true, nodeId, sensorId, sequence, timeMs, payloadLen,
    payload: bytes.slice(V2.HEADER_SIZE, V2.HEADER_SIZE + payloadLen),
  };
}

function decodeBnoPayload(payload) {
  if (payload.length < V2.BNO_SAMPLE_SIZE) return { ok: false };
  const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
  const values = {};
  for (let i = 0; i < BNO_COLUMNS.length; i += 1) {
    values[BNO_COLUMNS[i]] = dv.getFloat32(4 + i * 4, true);
  }
  return { ok: true, values, offsetUs: dv.getUint32(0, true) };
}

function decodeIcmPayload(payload) {
  if (payload.length >= V2.ICM_SAMPLE_SIZE_V4) {
    const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    const raw = [
      dv.getInt16(8, true), dv.getInt16(10, true), dv.getInt16(12, true),
      dv.getInt16(14, true), dv.getInt16(16, true), dv.getInt16(18, true),
    ];
    const values = {
      accel_x_g: raw[0] * 4.0 / 32768.0,
      accel_y_g: raw[1] * 4.0 / 32768.0,
      accel_z_g: raw[2] * 4.0 / 32768.0,
      gyro_x_dps: raw[3] * 2000.0 / 32768.0,
      gyro_y_dps: raw[4] * 2000.0 / 32768.0,
      gyro_z_dps: raw[5] * 2000.0 / 32768.0,
    };
    return { ok: true, values, offsetUs: dv.getUint32(0, true), sampleSequence: dv.getUint32(4, true) };
  }
  if (payload.length >= V2.ICM_SAMPLE_SIZE_V3) {
    const dv = new DataView(payload.buffer, payload.byteOffset, payload.byteLength);
    const raw = [
      dv.getInt16(0, true), dv.getInt16(2, true), dv.getInt16(4, true),
      dv.getInt16(6, true), dv.getInt16(8, true), dv.getInt16(10, true),
    ];
    const values = {
      accel_x_g: raw[0] * 4.0 / 32768.0,
      accel_y_g: raw[1] * 4.0 / 32768.0,
      accel_z_g: raw[2] * 4.0 / 32768.0,
      gyro_x_dps: raw[3] * 2000.0 / 32768.0,
      gyro_y_dps: raw[4] * 2000.0 / 32768.0,
      gyro_z_dps: raw[5] * 2000.0 / 32768.0,
    };
    return { ok: true, values, offsetUs: null };
  }
  return { ok: false };
}

/**
 * Per-stream health and per-node clock-offset tracking (design Sections 7.1, 10).
 *
 * Node time_ms values are independent millisecond counters. Each stream's
 * samples are mapped onto the master receive clock via a per-node offset,
 * estimated as the median of (nodeTimeMs - recvMs) pairs and re-estimated with
 * a rolling median that rejects transport jitter. The notebook's training
 * alignment treats node timestamps as directly comparable once boot offsets
 * are removed, which is exactly what this mapping reproduces.
 */
export class StreamHealth {
  constructor(nodeId, sensorId, targetRateHz = 25) {
    this.nodeId = nodeId;
    this.sensorId = sensorId;
    // The Master stamps every relayed envelope with ONE global sequence
    // counter shared across all streams, so per-stream sequence gaps count
    // other streams' frames and say nothing about loss. Loss is measured as
    // the arrival deficit against the contract rate instead.
    this.targetRateHz = targetRateHz;
    this.received = 0;
    this.lastSeq = null;
    this.lastRecvMs = null;
    this.lastNodeTimeMs = null;
    this.maxGapMs = 0;
    this.firstRecvMs = null;
    this.bytes = 0;
  }

  observe(sequence, nodeTimeMs, recvMs, bytes) {
    if (this.lastRecvMs !== null) {
      this.maxGapMs = Math.max(this.maxGapMs, recvMs - this.lastRecvMs);
    }
    this.lastSeq = sequence;
    this.lastRecvMs = recvMs;
    this.lastNodeTimeMs = nodeTimeMs;
    this.received += 1;
    this.bytes += bytes;
    if (this.firstRecvMs === null) this.firstRecvMs = recvMs;
  }

  /** Samples per second over the whole observed span. */
  averageRate() {
    if (this.received < 2 || this.firstRecvMs === null || this.lastRecvMs === null) return 0;
    const spanS = (this.lastRecvMs - this.firstRecvMs) / 1000;
    return spanS > 0 ? (this.received - 1) / spanS : 0;
  }

  /**
   * Fraction (0..1) of the contract-rate samples that never arrived: 1 minus
   * received/expected over the observed span. Null while too little data has
   * arrived to judge.
   */
  lossFraction() {
    if (this.received < 2 || this.firstRecvMs === null || this.lastRecvMs === null) return null;
    const spanS = (this.lastRecvMs - this.firstRecvMs) / 1000;
    if (spanS <= 0) return null;
    return Math.max(0, 1 - this.received / (spanS * this.targetRateHz));
  }

  staleMs(nowMs) {
    return this.lastRecvMs === null ? Infinity : nowMs - this.lastRecvMs;
  }
}

export class TimeBase {
  constructor() {
    // Per stream: `pairs` feeds a rolling-max offset estimate (see observe);
    // `history` stores the estimate every 500 ms so the skew gate can measure
    // how one stream's clock mapping drifts RELATIVE to the other streams —
    // the cross-node disagreement that actually corrupts the shared grid.
    this.offsets = new Map();
    for (const id of DISPLAY_SOURCE_IDS) {
      this.offsets.set(id, {
        pairs: [], offsetMs: null, driftMsPerMin: 0,
        history: [], lastHistoryMs: -Infinity,
      });
    }
  }

  observe(nodeId, nodeTimeMs, recvMs) {
    const state = this.offsets.get(nodeId);
    if (!state) return;
    // A raw sample = nodeTime - recv = trueOffset - linkDelay: it sits BELOW
    // the true clock offset, and queueing delay only pulls it further down.
    // The rolling maximum over ~2 s is the least-delayed recent sample; it
    // tracks both clock drift and delay changes within ~one window, keeping
    // each stream's mapped clock aligned with its present reality.
    state.pairs.push(nodeTimeMs - recvMs);
    if (state.pairs.length > 50) state.pairs.shift();
    let maxOffset = -Infinity;
    for (const value of state.pairs) {
      if (value > maxOffset) maxOffset = value;
    }
    state.offsetMs = maxOffset;

    if (recvMs - state.lastHistoryMs >= 500) {
      state.lastHistoryMs = recvMs;
      state.history.push({ atMs: recvMs, offsetMs: maxOffset });
      if (state.history.length > 24) state.history.shift();
      const oldest = state.history[0];
      state.driftMsPerMin = ((state.offsetMs - oldest.offsetMs) * 60000) / Math.max(1, recvMs - oldest.atMs);
    }
  }

  /** Map a node's time_ms onto the master receive clock. */
  mappedMs(nodeId, nodeTimeMs) {
    const state = this.offsets.get(nodeId);
    if (!state || state.offsetMs === null) return null;
    return nodeTimeMs - state.offsetMs;
  }

  /**
   * Estimated cross-node alignment error in ms (Section 10): how far this
   * stream's clock mapping has drifted over the last ~10 s RELATIVE to the
   * median drift of the model streams. A common bias cancels; a stream whose
   * link degrades differently from the others is what breaks the shared
   * grid. Null while history spans less than ~8 s.
   */
  skewMs(nodeId) {
    const modelStreams = NODE_IDS
      .map((id) => this.offsets.get(id))
      .filter((state) => state && state.history.length >= 2);
    if (modelStreams.length < NODE_IDS.length) return null;
    const nowMs = modelStreams[0].history[modelStreams[0].history.length - 1].atMs;
    const windowMs = 10000;
    const drifts = new Map();
    for (const id of NODE_IDS) {
      const state = this.offsets.get(id);
      let reference = null;
      for (const entry of state.history) {
        if (nowMs - entry.atMs <= windowMs) break;
        reference = entry;
      }
      if (!reference) return null; // not enough span yet
      drifts.set(id, state.offsetMs - reference.offsetMs);
    }
    const sorted = [...drifts.values()].sort((a, b) => a - b);
    const medianDrift = sorted[sorted.length >> 1];
    return Math.abs(drifts.get(nodeId) - medianDrift);
  }

  reset() {
    for (const state of this.offsets.values()) {
      state.pairs.length = 0;
      state.offsetMs = null;
      state.history.length = 0;
      state.lastHistoryMs = -Infinity;
      state.driftMsPerMin = 0;
    }
  }
}

export class BleTransport {
  constructor({ onSample, onLog }) {
    this.onSample = onSample;
    this.onLog = onLog || (() => {});
    this.device = null;
    this.server = null;
    this.imuChar = null;
    this.cmdChar = null;
    this.pipeSeq = 1;
    this.health = new Map(); // key "n{node}s{sensor}" -> StreamHealth
    this.timeBase = new TimeBase();
    this.rxBytes = 0;
    this.connected = false;
  }

  static streamKey(nodeId, sensorId) {
    return `n${nodeId}s${sensorId}`;
  }

  /**
   * Preflight the things that make the chooser come up empty, and say which
   * one failed. Web Bluetooth reports "no devices found" identically whether
   * the adapter is off, the page is not a secure context, or the Master simply
   * is not advertising, so check what we can before opening the chooser.
   */
  async assertBluetoothReady() {
    if (!window.isSecureContext) {
      throw new Error(
        `page is not a secure context (${window.location.origin}). Web Bluetooth ` +
          "needs http://localhost, http://127.0.0.1 or https. Serving on :: and " +
          "browsing to a LAN address or [::] will not work."
      );
    }
    if (!navigator.bluetooth) {
      throw new Error(
        "navigator.bluetooth is unavailable. Use Chrome or Edge on desktop; " +
          "Firefox and Safari do not implement Web Bluetooth."
      );
    }
    if (navigator.bluetooth.getAvailability) {
      const available = await navigator.bluetooth.getAvailability();
      if (!available) {
        throw new Error("no Bluetooth adapter available, or Bluetooth is turned off in the OS.");
      }
    }
  }

  /**
   * @param {{acceptAll?: boolean}} options acceptAll drops the name filter and
   * lists every nearby device. Use it when the Master does not appear: a
   * shortened or missing local name in the advertisement makes a namePrefix
   * filter hide a device that is otherwise perfectly connectable.
   */
  async connect({ acceptAll = false } = {}) {
    await this.assertBluetoothReady();
    const request = acceptAll
      ? { acceptAllDevices: true, optionalServices: [BLE_CFG.serviceUuid] }
      : {
          filters: [
            { namePrefix: BLE_CFG.deviceNamePrefix },
            { namePrefix: BLE_CFG.deviceNamePrefixFallback },
          ],
          optionalServices: [BLE_CFG.serviceUuid],
        };
    let device;
    try {
      device = await navigator.bluetooth.requestDevice(request);
    } catch (err) {
      if (err && err.name === "NotFoundError") {
        // Chrome uses NotFoundError both for "you cancelled" and "nothing matched".
        throw new Error(
          `${err.message} — if the chooser was empty: the Master advertises as ` +
            `"${BLE_CFG.deviceNamePrefix}" only while it is NOT already connected, so close ` +
            "any other page or app holding it, power-cycle the Master, and retry. " +
            (acceptAll ? "" : 'If it still does not appear, use "Scan All Devices".')
        );
      }
      throw err;
    }
    if (device.name && !device.name.startsWith(BLE_CFG.deviceNamePrefixFallback)) {
      this.onLog(
        `Selected "${device.name}", which is not a ${BLE_CFG.deviceNamePrefixFallback}* device. ` +
          "Continuing, but the service lookup will fail if this is not the Master.",
        "warn"
      );
    }
    this.device = device;
    device.addEventListener("gattserverdisconnected", () => {
      this.connected = false;
      this.onLog("Master disconnected.", "error");
      this.onDisconnect && this.onDisconnect();
    });
    this.onLog(`Connecting to ${device.name || "Master"}...`);
    this.server = await device.gatt.connect();
    const service = await this.server.getPrimaryService(BLE_CFG.serviceUuid);
    this.imuChar = await service.getCharacteristic(BLE_CFG.imuCharUuid);
    this.cmdChar = await service.getCharacteristic(BLE_CFG.cmdCharUuid);
    await this.imuChar.startNotifications();
    this.imuChar.addEventListener("characteristicvaluechanged", (evt) => this.onNotify(evt));
    let ackChar = null;
    try {
      ackChar = await service.getCharacteristic(BLE_CFG.cmdAckCharUuid);
      await ackChar.startNotifications();
      ackChar.addEventListener("characteristicvaluechanged", (evt) => this.onAckNotify(evt));
    } catch (err) {
      this.onLog(`CMD_ACK notifications unavailable: ${err.message}`, "warn");
    }
    try {
      // PIPESTATUSTX lives inside the pipe service, not a service of its own.
      const statusChar = await service.getCharacteristic(BLE_CFG.statusCharUuid);
      await statusChar.startNotifications();
      statusChar.addEventListener("characteristicvaluechanged", (evt) => this.onStatusNotify(evt));
    } catch (err) {
      this.onLog(`Status notifications unavailable: ${err.message}`, "warn");
    }
    this.connected = true;
    this.onLog("Master connected.");
  }

  disconnect() {
    if (this.device && this.device.gatt && this.device.gatt.connected) {
      this.device.gatt.disconnect();
    }
    this.connected = false;
  }

  async writeCommand(bytes) {
    if (!this.cmdChar) throw new Error("Command characteristic unavailable");
    const raw = bytes instanceof Uint8Array ? bytes : Uint8Array.from(bytes);
    if (raw.length === 0 || raw.length > BLE_CFG.maxPayloadLen) {
      throw new Error(`Command length ${raw.length} outside 1..${BLE_CFG.maxPayloadLen}`);
    }
    await this.cmdChar.writeValueWithoutResponse(raw);
  }

  async sendPipeCommand(payload, msgType = BLEPIPE.MSG_COMMAND, dstId = BLEPIPE.DST_BROADCAST) {
    const packet = blepipeEncode(msgType, payload, dstId, this.pipeSeq);
    this.pipeSeq = (this.pipeSeq + 1) >>> 0;
    await this.writeCommand(packet);
  }

  async startStream(intervalMs) {
    // Silence the Master's own IMU preview first: it is display-only for this
    // tool, and on the shared link it crowds out the six model streams.
    await this.sendPipeCommand([CMD.MASTER_OWN_STREAM, 0], BLEPIPE.MSG_STREAM_CONTROL);
    await this.sendPipeCommand([CMD.SET_INTERVAL, intervalMs & 0xff], BLEPIPE.MSG_STREAM_CONTROL);
    await this.sendPipeCommand([CMD.START_STREAM], BLEPIPE.MSG_STREAM_CONTROL);
    this.onLog(`Stream start sent: interval_ms=${intervalMs}, master own-stream off.`);
  }

  /** 0xA2 alone: change the interval without restarting the stream. */
  async setStreamInterval(intervalMs) {
    await this.sendPipeCommand(
      [CMD.SET_INTERVAL, intervalMs & 0xff],
      BLEPIPE.MSG_STREAM_CONTROL
    );
    this.onLog(`Stream interval set: ${intervalMs} ms.`);
  }

  async stopStream() {
    await this.sendPipeCommand([CMD.STOP_STREAM], BLEPIPE.MSG_STREAM_CONTROL);
    // Restore the Master's own preview so the desktop tool behaves as before.
    await this.sendPipeCommand([CMD.MASTER_OWN_STREAM, 1], BLEPIPE.MSG_STREAM_CONTROL);
    this.onLog("Stream stop sent.");
  }

  async motorPercent(nodeId, percent) {
    if (!NODE_IDS.includes(nodeId)) throw new Error(`Invalid haptic target node ${nodeId}`);
    const level = Math.max(0, Math.min(100, Math.round(percent)));
    await this.sendPipeCommand([CMD.SET_ERM, level], BLEPIPE.MSG_COMMAND, nodeId);
  }

  /**
   * Section 6.4 bounded pulse. The Node owns the stop deadline, so no
   * follow-up off-command is required and a dropped packet cannot leave a
   * motor running. Values outside the bounds are rejected here and again by
   * the Node rather than clamped.
   */
  async hapticPulse(nodeId, intensityPercent, durationMs, eventId) {
    if (!NODE_IDS.includes(nodeId)) throw new Error(`Invalid haptic target node ${nodeId}`);
    const intensity = Math.round(intensityPercent);
    const duration = Math.round(durationMs);
    if (intensity < PULSE_BOUNDS.minIntensityPercent || intensity > PULSE_BOUNDS.maxIntensityPercent) {
      throw new Error(`Pulse intensity ${intensity}% outside 1-100%`);
    }
    if (duration < PULSE_BOUNDS.minDurationMs || duration > PULSE_BOUNDS.maxDurationMs) {
      throw new Error(`Pulse duration ${duration} ms outside 50-500 ms`);
    }
    const id = eventId >>> 0;
    await this.sendPipeCommand(
      [
        CMD.HAPTIC_PULSE,
        intensity,
        duration & 0xff,
        (duration >> 8) & 0xff,
        id & 0xff,
        (id >> 8) & 0xff,
        (id >> 16) & 0xff,
        (id >> 24) & 0xff,
      ],
      BLEPIPE.MSG_COMMAND,
      nodeId
    );
  }

  /** 0xA4 passive buzzer, 0..99 %. Master accepts it for itself, nodes relay. */
  async buzzerPercent(sourceId, percent) {
    const level = Math.max(0, Math.min(99, Math.round(percent)));
    await this.sendPipeCommand([CMD.SET_BUZZER, level], BLEPIPE.MSG_COMMAND, this.dstFor(sourceId));
  }

  /** 0xA5 RGB mask, bit0=R bit1=G bit2=B. */
  async rgbMask(sourceId, mask) {
    await this.sendPipeCommand(
      [CMD.SET_RGB, mask & 0x07],
      BLEPIPE.MSG_COMMAND,
      this.dstFor(sourceId)
    );
  }

  /** 0xA5 with bit7 set releases the BLE actuator override on that source. */
  async clearActuatorOverride(sourceId) {
    await this.sendPipeCommand(
      [CMD.SET_RGB, RGB_OVERRIDE_OFF],
      BLEPIPE.MSG_COMMAND,
      this.dstFor(sourceId)
    );
  }

  /** 0xA6 touch feedback self-test (buzzer + RGB burst). */
  async touchTest(sourceId) {
    await this.sendPipeCommand([CMD.TOUCH_TEST, 0], BLEPIPE.MSG_COMMAND, this.dstFor(sourceId));
  }

  /** 0xB2 force rediscovery of all nodes (Master-scoped). */
  async rediscoverNodes() {
    await this.sendPipeCommand([CMD.REDISCOVER], BLEPIPE.MSG_COMMAND, BLEPIPE.DST_BROADCAST);
  }

  /** 0xB4 request the Master's discovered-node report. */
  async requestDiscoveredNodes() {
    await this.sendPipeCommand([CMD.DISCOVERED_NODES], BLEPIPE.MSG_COMMAND, BLEPIPE.DST_BROADCAST);
  }

  /** 0xB1 query a node's stored id. */
  async queryNodeId(nodeId) {
    await this.sendPipeCommand([CMD.QUERY_NODE_ID, nodeId & 0xff], BLEPIPE.MSG_COMMAND, BLEPIPE.DST_BROADCAST);
  }

  /**
   * 0xB3 erase retained un-uploaded sessions. DESTRUCTIVE: any node flash copy
   * that was never collected is discarded. A node holding a retained session
   * sits in transfer_priority and emits no live samples at all, so this is the
   * escape hatch when a transfer genuinely failed.
   */
  async resetRetainedSessions() {
    await this.sendPipeCommand([CMD.RESET_RETAINED], BLEPIPE.MSG_COMMAND, BLEPIPE.DST_BROADCAST);
  }

  /** Broadcast for the Master itself, unicast for a node. */
  dstFor(sourceId) {
    return sourceId === MASTER_ID ? BLEPIPE.DST_BROADCAST : sourceId;
  }

  onNotify(evt) {
    const view = evt.target.value;
    const data = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
    this.rxBytes += data.length;
    const recvMs = performance.now();
    // Reliable-transfer frames share this characteristic on some builds; a
    // record-lane frame never starts with 0xB1, so demux defensively.
    if (data.length > 0 && data[0] !== 0xb1) return;
    const parsed = parseEnvelopeV2(data);
    if (!parsed.ok) return;
    if (!DISPLAY_SOURCE_IDS.includes(parsed.nodeId)) return;

    let decoded;
    if (parsed.sensorId === SENSOR.BNO) decoded = decodeBnoPayload(parsed.payload);
    else if (parsed.sensorId === SENSOR.ICM) decoded = decodeIcmPayload(parsed.payload);
    if (!decoded || !decoded.ok) return;

    const key = BleTransport.streamKey(parsed.nodeId, parsed.sensorId);
    let health = this.health.get(key);
    if (!health) {
      health = new StreamHealth(parsed.nodeId, parsed.sensorId);
      this.health.set(key, health);
    }
    health.observe(parsed.sequence, parsed.timeMs, recvMs, data.length);
    this.timeBase.observe(parsed.nodeId, parsed.timeMs, recvMs);

    const mappedMs = this.timeBase.mappedMs(parsed.nodeId, parsed.timeMs);
    this.onSample({
      nodeId: parsed.nodeId,
      sensorId: parsed.sensorId,
      // Only the three arm nodes feed the model; MASTER and N1 are display-only
      // and must never degrade inference by their absence.
      isModelStream: NODE_IDS.includes(parsed.nodeId),
      sequence: parsed.sequence,
      nodeTimeMs: parsed.timeMs,
      recvMs,
      mappedMs: mappedMs === null ? recvMs : mappedMs,
      timeBaseReady: mappedMs !== null,
      values: decoded.values,
    });
  }

  onAckNotify(evt) {
    const view = evt.target.value;
    const data = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
    const pipe = blepipeDecode(data);
    if (!pipe.ok) return;
    const cmd = pipe.payload[0];
    const name = CMD_NAME[cmd] ?? `0x${(cmd ?? 0).toString(16)}`;
    if (pipe.msgType === BLEPIPE.MSG_NACK) {
      this.onLog(`Master NACK: ${name} rejected`, "warn");
    } else if (pipe.msgType === BLEPIPE.MSG_ACK) {
      this.onLog(`Master ACK: ${name}`);
    }
  }

  onStatusNotify(evt) {
    const view = evt.target.value;
    const data = new Uint8Array(view.buffer, view.byteOffset, view.byteLength);
    const pipe = blepipeDecode(data);
    if (!pipe.ok) return;
    if (pipe.msgType === BLEPIPE.MSG_TOPOLOGY && pipe.payload.length >= 1) {
      const count = pipe.payload[0] || 0;
      const ids = [];
      for (let i = 0; i < count && 1 + i < pipe.payload.length; i += 1) ids.push(pipe.payload[1 + i]);
      this.onLog(`Topology: ${ids.length ? ids.map((id) => `NODE${id}`).join(", ") : "none"}`);
      this.onTopology && this.onTopology(ids);
    } else if (pipe.msgType === BLEPIPE.MSG_LOG && pipe.payload.length > 0) {
      // SWO-free firmware telemetry: the Master emits an ASCII "LIVE ..." line
      // ~1 Hz while streaming so the pipeline can be debugged from this page.
      let text = "";
      for (let i = 0; i < pipe.payload.length; i += 1) {
        const c = pipe.payload[i];
        if (c === 0) break;
        text += String.fromCharCode(c);
      }
      if (text) this.onLog(`[master] ${text}`);
    }
  }

  streams() {
    return [...this.health.values()];
  }

  resetSessionState() {
    this.health.clear();
    this.timeBase.reset();
    this.rxBytes = 0;
  }
}

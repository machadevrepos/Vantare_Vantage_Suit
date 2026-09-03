/**
 * Parity harness runner (design Section 12, Python-to-browser parity test).
 *
 * Reads a fixture JSON file produced by
 * host/tests/python/test_live_preprocessing_parity.py, feeds every sample
 * through the browser Preprocessor in arrival order, and prints the emitted
 * windows as JSON on stdout so the Python side can compare feature vectors.
 *
 *   node host/tests/scripts/run_preprocessing_fixture.mjs <fixture.json>
 */
import { readFileSync } from "node:fs";
import { Preprocessor } from "../../live_tool/js/ml-preprocessing.js";

const fixturePath = process.argv[2];
if (!fixturePath) {
  process.stderr.write("usage: run_preprocessing_fixture.mjs <fixture.json>\n");
  process.exit(2);
}

const fixture = JSON.parse(readFileSync(fixturePath, "utf8"));
const preprocessor = new Preprocessor(fixture.contract);
const emitted = [];

for (const sample of fixture.samples) {
  preprocessor.pushSample(sample.node, sample.sensor, sample.t, sample.seq, sample.values);
  // Drain every window this sample made available, mirroring main.js.
  for (;;) {
    const result = preprocessor.maybeEmit();
    if (result.status === "emitted") {
      emitted.push({
        startS: result.window.startS,
        endS: result.window.endS,
        features: Array.from(result.window.features),
      });
      continue;
    }
    if (result.status === "invalid") {
      emitted.push({ invalid: result.reason });
      continue;
    }
    break;
  }
}

process.stdout.write(JSON.stringify({ windows: emitted }));

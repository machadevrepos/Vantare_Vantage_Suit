/**
 * Rep analyzer harness runner.
 *
 * Reads a fixture of motion packets produced by
 * host/tests/python/test_rep_analyzer.py, feeds them through RepAnalyzer in
 * order, and prints the completed reps and session summary as JSON.
 *
 *   node host/tests/scripts/run_rep_fixture.mjs <fixture.json>
 */
import { readFileSync } from "node:fs";
import { RepAnalyzer } from "../../live_tool/js/rep-analyzer.js";

const fixturePath = process.argv[2];
if (!fixturePath) {
  process.stderr.write("usage: run_rep_fixture.mjs <fixture.json>\n");
  process.exit(2);
}

const fixture = JSON.parse(readFileSync(fixturePath, "utf8"));
const results = [];

for (const scenario of fixture.scenarios) {
  const emitted = [];
  const analyzer = new RepAnalyzer({
    params: scenario.params || {},
    target: scenario.target || {},
    onRep: (rep) => emitted.push(rep),
  });
  for (const frame of scenario.frames) analyzer.pushFrame(frame);
  results.push({
    name: scenario.name,
    emitted,
    reps: analyzer.reps,
    summary: analyzer.summary,
    inProgress: analyzer.current !== null,
  });
}

process.stdout.write(JSON.stringify({ scenarios: results }));

/**
 * Billboard math for the full-body avatar.
 *
 *   node host/tests/scripts/test_avatar_billboard.mjs
 *
 * The figure is built from flat CSS cards. Seen from a three-quarter camera, a
 * limb card pointing forward went edge-on (a hairline) and its labels mirrored
 * (field feedback 2026-09-11). Limb surfaces now turn about their own long axis
 * toward the camera, and labels cancel every rotation above them. These tests
 * pin both properties for arbitrary camera and limb orientations.
 */
import assert from "node:assert/strict";
import {
  limbBillboard,
  multiplyRotations,
  rotationFromCssTransform,
  rotationMatrixForQuaternion,
  transposeRotation,
} from "../../live_tool/js/arm-avatar.js";

const apply = (m, v) => [0, 1, 2].map((r) => m[r] * v[0] + m[4 + r] * v[1] + m[8 + r] * v[2]);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (v) => { const n = Math.hypot(...v); return v.map((c) => c / n); };
const Q = (axis, deg) => {
  const n = Math.hypot(...axis);
  const h = (deg * Math.PI) / 360;
  const s = Math.sin(h);
  return { qx: (axis[0] / n) * s, qy: (axis[1] / n) * s, qz: (axis[2] / n) * s, qw: Math.cos(h) };
};
const close = (a, b, tol, msg) => assert.ok(Math.abs(a - b) < tol, `${msg}: ${a} vs ${b}`);

// A camera like the body rig's: tilted and turned.
const camera = multiplyRotations(
  rotationMatrixForQuaternion(Q([1, 0, 0], -8)),
  rotationMatrixForQuaternion(Q([0, 1, 0], 35))
);
const towardCameraRig = apply(transposeRotation(camera), [0, 0, 1]);

const limbs = {
  hanging: Q([1, 0, 0], 0),
  forward: Q([1, 0, 0], 90),
  side: Q([0, 0, -1], 90),
  diagonal: Q([0.6, 0.2, -0.77], 71),
  twisted: Q([0.1, 0.99, 0.05], 130),
};

for (const [name, q] of Object.entries(limbs)) {
  const limb = rotationMatrixForQuaternion(q);
  const local = apply(transposeRotation(limb), towardCameraRig);
  const board = limbBillboard(local);
  assert.ok(board, `${name}: billboard expected`);
  const world = multiplyRotations(camera, multiplyRotations(limb, board));
  const cardNormal = norm(apply(world, [0, 0, 1]));
  const limbAxis = norm(apply(world, [0, 1, 0]));
  // The card must face the camera as squarely as a turn about the limb allows:
  // its normal is the camera direction with the along-limb part removed.
  const along = limbAxis[2];
  const best = Math.sqrt(Math.max(0, 1 - along * along));
  close(cardNormal[2], best, 1e-6, `${name}: card faces the camera`);
  // And the turn must be about the limb's own axis, never tilting it.
  const unturned = norm(apply(multiplyRotations(camera, limb), [0, 1, 0]));
  close(dot(limbAxis, unturned), 1, 1e-6, `${name}: limb axis unchanged`);

  // Label = inverse of everything above it: flat and upright on screen.
  const label = transposeRotation(multiplyRotations(multiplyRotations(camera, limb), board));
  const labelWorld = multiplyRotations(multiplyRotations(multiplyRotations(camera, limb), board), label);
  for (let i = 0; i < 3; i += 1) {
    const axis = [0, 0, 0];
    axis[i] = 1;
    close(dot(apply(labelWorld, axis), axis), 1, 1e-6, `${name}: label axis ${i} screen-aligned`);
  }
}

// A limb pointing straight at the camera has no best turn: the caller keeps
// the previous one rather than snapping to an arbitrary angle.
assert.equal(limbBillboard([0, 1, 0]), null);

// Computed-style parsing: scale is stripped, 2D matrices are accepted.
{
  const r = rotationFromCssTransform("matrix3d(0.5,0,0,0, 0,0.5,0,0, 0,0,0.5,0, 10,20,0,1)");
  assert.deepEqual(r.slice(0, 11), [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  const two = rotationFromCssTransform("matrix(0.85, 0, 0, 0.85, 0, 0)");
  assert.deepEqual([two[0], two[5], two[10]], [1, 1, 1]);
  assert.deepEqual(rotationFromCssTransform("none")[0], 1);
}

console.log("Avatar billboard tests passed");

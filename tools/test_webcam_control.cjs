const test = require('node:test');
const assert = require('node:assert/strict');
const webcam = require('../web/webcam-control.js');
const hand = require('../web/hand-follow-policy.js');

test('neutral pose is stable and hand motion changes the expected axes', () => {
  const anchor = {x:.5, y:.5};
  assert.deepEqual(webcam.target(anchor, anchor), webcam.middle);
  const right = webcam.target({x:.6, y:.5}, anchor);
  const up = webcam.target({x:.5, y:.4}, anchor);
  assert.ok(right[0] < webcam.middle[0]);
  assert.ok(up[1] < webcam.middle[1]);
  assert.ok(up[2] > webcam.middle[2]);
  assert.equal(webcam.target({x:NaN, y:.5}, anchor), null);
});

test('reanchoring cannot accumulate beyond visual joint limits and motion is rate limited', () => {
  const target = webcam.target({x:1,y:0}, {x:0,y:1});
  const limited = webcam.boundedFromAnchor(target, [3,3,3,3,3,3]);
  assert.deepEqual(limited, [1.2,1.6,1.8,1.2,1.2,1.6]);
  const advanced = webcam.step([0,0], [1,-1], .05);
  assert.ok(Math.abs(advanced[0] - .04) < 1e-8);
  assert.ok(Math.abs(advanced[1] + .04) < 1e-8);
});

test('pinch mapping closes and reopens with hysteresis', () => {
  const pinch = new hand.PinchGripper();
  assert.equal(pinch.observe(.2), 0);
  assert.equal(pinch.observe(.35), 0);
  assert.ok(pinch.observe(.8) > .07);
  assert.equal(pinch.observe(null), null);
});


test('both opposing fingers open through the full positive URDF travel', () => {
  assert.deepEqual(webcam.gripperJointValues(0), [0, 0]);
  assert.deepEqual(webcam.gripperJointValues(.0715), [.05, .05]);
  assert.deepEqual(webcam.gripperJointValues(.2), [.05, .05]);
  assert.equal(webcam.gripperJointValues(NaN), null);
});

test('small stationary landmark noise is held and deliberate movement follows', () => {
  const filter = new webcam.InputFilter();
  const start = filter.hand({x:.5, y:.5}, 0);
  for (let i = 1; i <= 50; i++) {
    const noise = i % 2 ? .002 : -.002;
    assert.deepEqual(filter.hand({x:.5 + noise, y:.5 - noise}, i * 33), start);
  }
  let moved;
  for (let i = 51; i <= 70; i++) moved = filter.hand({x:.58, y:.45}, i * 33);
  assert.ok(moved.x > .57 && moved.y < .46);
  filter.reset();
  assert.deepEqual(filter.hand({x:.8, y:.3}, 2400), {x:.8, y:.3});
});

test('gripper noise is suppressed while retaining full open and closed positions', () => {
  const filter = new webcam.InputFilter();
  assert.equal(filter.gripper(.035, 0), .035);
  for (let i = 1; i <= 30; i++) {
    const noise = i % 2 ? .0004 : -.0004;
    assert.equal(filter.gripper(.035 + noise, i * 33), .035);
  }
  let grip;
  for (let i = 31; i <= 55; i++) grip = filter.gripper(0, i * 33);
  assert.equal(grip, 0);
  for (let i = 56; i <= 90; i++) grip = filter.gripper(.0715, i * 33);
  assert.equal(grip, .0715);
});

test('joint motion eases into movement, reverses smoothly and settles without overshoot', () => {
  const motion = new webcam.MotionSmoother([0]);
  let previous = 0, firstDelta = 0;
  for (let i = 0; i < 180; i++) {
    const position = motion.update([.4], 1 / 60)[0];
    const delta = position - previous;
    assert.ok(delta >= 0 && delta <= .8 / 60 + 1e-12);
    if (i === 0) firstDelta = delta;
    previous = position;
  }
  assert.ok(firstDelta < .8 / 60 / 2);
  assert.ok(Math.abs(previous - .4) < 1e-8);
  for (let i = 0; i < 180; i++) {
    const position = motion.update([-.2], 1 / 60)[0];
    assert.ok(position <= previous + 1e-12 && position >= -.2 - 1e-12);
    previous = position;
  }
  assert.ok(Math.abs(previous + .2) < 1e-8);
  motion.reset([.1]);
  assert.deepEqual(motion.update([.1], 1 / 60), [.1]);
});

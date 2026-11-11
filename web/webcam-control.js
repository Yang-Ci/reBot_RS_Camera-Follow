(function (root) {
  'use strict';
  const RAD = Math.PI / 180;
  const middle = [-24.712, 19.741, 21.147, 5.375, .105, -.088].map(v => v * RAD);
  const clamp = (value, lo, hi) => Math.max(lo, Math.min(hi, value));
  // Time-based smoothing plus a small output deadband. Faster changes receive
  // less filtering; the filter is reset whenever the hand must be reanchored.
  class InputFilter {
    constructor() { this.reset(); }
    reset() { this.point = null; this.output = null; this.time = null; this.grip = null; this.gripOutput = null; this.gripTime = null; }
    hand(sample, time) {
      if (!sample || !Number.isFinite(sample.x) || !Number.isFinite(sample.y) || !Number.isFinite(time)) return null;
      if (!this.point || this.time === null || time <= this.time || time - this.time > 400) {
        this.point = {x: sample.x, y: sample.y}; this.output = {...this.point}; this.time = time;
        return {...this.output};
      }
      const dt = Math.min(.12, (time - this.time) / 1000); this.time = time;
      const displacement = Math.hypot(sample.x - this.point.x, sample.y - this.point.y);
      const tau = displacement > .035 ? .055 : .13;
      const alpha = 1 - Math.exp(-dt / tau);
      this.point.x += (sample.x - this.point.x) * alpha;
      this.point.y += (sample.y - this.point.y) * alpha;
      if (Math.hypot(this.point.x - this.output.x, this.point.y - this.output.y) > .003) this.output = {...this.point};
      return {...this.output};
    }
    gripper(value, time) {
      if (!Number.isFinite(value) || !Number.isFinite(time)) return null;
      if (this.grip === null || this.gripTime === null || time <= this.gripTime || time - this.gripTime > 400) {
        this.grip = value; this.gripOutput = value; this.gripTime = time; return value;
      }
      const dt = Math.min(.12, (time - this.gripTime) / 1000); this.gripTime = time;
      const tau = Math.abs(value - this.grip) > .012 ? .065 : .14;
      this.grip += (value - this.grip) * (1 - Math.exp(-dt / tau));
      if (Math.abs(this.grip - this.gripOutput) > .001 || (value === 0 && this.grip < .001) || (value >= .0715 && this.grip > .0705)) {
        this.gripOutput = value === 0 && this.grip < .001 ? 0 : value >= .0715 && this.grip > .0705 ? .0715 : this.grip;
      }
      return this.gripOutput;
    }
  }
  // Critically damped motion avoids the abrupt velocity reversals caused by
  // applying a speed limit directly to changing landmark targets.
  class MotionSmoother {
    constructor(initial) { this.reset(initial); }
    reset(initial) { this.position = initial.slice(); this.velocity = initial.map(() => 0); }
    update(target, dt) {
      if (!Number.isFinite(dt) || dt <= 0 || !Array.isArray(target) ||
          target.length !== this.position.length || !target.every(Number.isFinite)) return this.position.slice();
      const step = Math.min(dt, .1), omega = 11, limit = .8;
      const decay = Math.exp(-omega * step);
      this.position = this.position.map((x, i) => {
        const error = x - target[i], v = this.velocity[i];
        const term = v + omega * error;
        const next = target[i] + (error + term * step) * decay;
        const nextV = (v - omega * term * step) * decay;
        const movement = clamp(next - x, -limit * step, limit * step);
        this.velocity[i] = clamp(nextV, -limit, limit);
        return x + movement;
      });
      return this.position.slice();
    }
  }
  function target(sample, anchor) {
    if (!sample || !anchor || ![sample.x, sample.y, anchor.x, anchor.y].every(Number.isFinite)) return null;
    const x = clamp((sample.x - anchor.x) / .28, -1, 1);
    const y = clamp((anchor.y - sample.y) / .28, -1, 1);
    // Visual-only joint mapping. It does not solve Cartesian IK or drive a robot.
    return [middle[0] - .65 * x, middle[1] - .32 * y, middle[2] + .45 * y,
      middle[3] + .20 * y, middle[4], middle[5]];
  }
  function gripperJointValues(width) {
    if (!Number.isFinite(width)) return null;
    const travel = clamp(width / .0715, 0, 1) * .05;
    // The URDF gives both fingers positive travel with opposing local axes.
    return [travel, travel];
  }
  function boundedFromAnchor(goal, base) {
    const limits = [[-1.2,1.2],[.10,1.6],[.15,1.8],[-1.0,1.2],[-1.2,1.2],[-1.6,1.6]];
    return goal.map((value, i) => clamp(value - middle[i] + base[i], limits[i][0], limits[i][1]));
  }
  function step(current, desired, dt, maxSpeed = .8) {
    if (!Number.isFinite(dt) || dt <= 0) return current.slice();
    const delta = maxSpeed * Math.min(dt, .1);
    return current.map((q, i) => q + clamp(desired[i] - q, -delta, delta));
  }
  const api = { middle, InputFilter, MotionSmoother, target, gripperJointValues, boundedFromAnchor, step };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.WebcamControl = api;
})(typeof window !== 'undefined' ? window : globalThis);

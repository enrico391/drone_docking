// Execute the shipped dashboard script with a minimal DOM; no browser packages required.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const {script, parameters} = JSON.parse(fs.readFileSync(0, 'utf8'));
const elements = new Map();
const canvasContext = new Proxy({}, {get: (target, name) => target[name] ?? (() => {})});
class Element {
  constructor(tag) {
    this.tagName = tag;
    this.children = [];
    this.value = '';
    this.textContent = '';
    this.disabled = false;
    this.width = 560;
    this.height = 240;
  }
  set value(value) { this._value = String(value); }
  get value() { return this._value; }
  set innerHTML(html) {
    this.html = html;
    for (const match of html.matchAll(/<(\w+)\b[^>]*\bid="([^"]+)"[^>]*>/g)) {
      const element = new Element(match[1]);
      element.disabled = /\bdisabled\b/.test(match[0]);
      if (match[1] === 'select') element.value = 'base';
      elements.set(match[2], element);
    }
  }
  appendChild(child) { this.children.push(child); }
  setAttribute(name, value) { this[name] = value; }
  getContext() { return canvasContext; }
}
for (const id of ['axis-controls', 'readings', 'plots', 'pose-status', 'download',
                  'position-scale', 'yaw-scale', 'clear', 'scale-status', 'connection']) {
  elements.set(id, new Element('div'));
}
const requests = [];
const pending = [];
const context = vm.createContext({
  document: {
    getElementById: id => elements.get(id) ?? null,
    createElement: tag => new Element(tag),
  },
  fetch: async (url, options) => {
    if (url === '/api/state') return {ok: true, json: async () => ({connected: false})};
    assert.equal(url, '/api/gains');
    requests.push(JSON.parse(options.body));
    return new Promise(resolve => pending.push(resolve));
  },
  AbortSignal: {timeout: () => undefined},
  setTimeout: () => {},
});
vm.runInContext(script + `
globalThis.dashboard = {
  editors, applyGains,
  setTelemetry(value) { latest=value; fill(); render(); },
};`, context);
const $ = id => {
  assert.ok(elements.has(id), `missing control ${id}`);
  return elements.get(id);
};
const flush = () => new Promise(resolve => setImmediate(resolve));
const sample = () => ({
  parameters: {...parameters, pid_gain_far_y_kp: 0.9},
  axes: Object.fromEntries(['x', 'y', 'z', 'yaw'].map(a => [a, {kp: 0.1, ki: 0.2, kd: 0.3}])),
  measured_pose: {x: 0.1, y: 0.2, z: 3, yaw: 0},
  error: {x: 0.1, y: 0.2, z: null, yaw: 0},
  pose_valid: true, dock_age_s: 0.1, gain_near_distance_m: 1, gain_far_distance_m: 5,
});
async function main() {
  await flush();
  assert.equal($('axis-controls').children.length, 4);
  assert.equal($('plots').children.length, 4);
  assert.equal($('readings').children.length, 4);
  assert.ok(!elements.has('axis'));
  for (const a of ['x', 'y', 'z', 'yaw']) assert.equal($(`apply-${a}`).disabled, true);
  context.dashboard.setTelemetry(sample());
  for (const a of ['x', 'y', 'z', 'yaw']) {
    assert.equal($(`kp-${a}`).value, '0.1');
    assert.equal($(`apply-${a}`).disabled, false);
    assert.ok($(`effective-${a}`).textContent.includes(`Effective ${a.toUpperCase()}:`));
  }

  // Polling and changing Y's bank must not overwrite an unsaved X edit.
  $('kp-x').value = '0.72';
  $('kp-x').oninput();
  $('bank-y').value = 'far';
  $('bank-y').onchange();
  context.dashboard.setTelemetry(sample());
  assert.equal($('kp-x').value, '0.72');
  assert.equal($('kp-y').value, '0.9');
  assert.equal($('kp-y').min, -1);
  assert.equal($('kp-x').min, 0);
  assert.equal($('bank-z').value, 'base');

  // Updates can overlap across axes but each request remains axis-specific.
  const xUpdate = $('apply-x').onclick();
  assert.equal($('apply-x').disabled, true);
  assert.equal($('kp-x').disabled, true);
  assert.equal($('bank-x').disabled, true);
  assert.equal($('apply-y').disabled, false);
  await context.dashboard.applyGains('x');
  assert.equal(requests.length, 1, 'duplicate X request while already applying');
  $('kp-y').value = '-1';
  $('kp-y').oninput();
  const yUpdate = $('apply-y').onclick();
  assert.equal(requests.length, 2);
  assert.deepEqual(requests[0], {axis: 'x', bank: 'base', kp: 0.72, ki: 0.1, kd: 0.1});
  assert.deepEqual(requests[1], {axis: 'y', bank: 'far', kp: -1, ki: 0.1, kd: 0.1});
  pending[0]({ok: true, json: async () => ({ok: true, message: 'X applied'})});
  await xUpdate;
  assert.equal(context.dashboard.editors.x.dirty, false);
  assert.equal(context.dashboard.editors.y.dirty, true);
  assert.equal($('apply-y').disabled, true);
  assert.equal($('gain-status-x').textContent, 'X applied');
  pending[1]({ok: false, json: async () => ({ok: false, message: 'Y rejected'})});
  await yUpdate;
  assert.equal($('gain-status-y').textContent, 'Y rejected');
  assert.equal($('gain-status-x').textContent, 'X applied');
  assert.equal(context.dashboard.editors.y.dirty, true);
  assert.equal($('kp-y').value, '-1');
  assert.equal($('apply-y').disabled, false);

  // Base-bank validation is local to the edited axis, and disconnect disables all updates.
  $('kp-z').value = '-1';
  $('kp-z').oninput();
  await $('apply-z').onclick();
  assert.equal(requests.length, 2);
  assert.ok($('gain-status-z').textContent.includes('Use nonnegative gains'));
  assert.equal($('gain-status-x').textContent, 'X applied');
  assert.equal($('download').disabled, false);
  context.dashboard.setTelemetry(null);
  for (const a of ['x', 'y', 'z', 'yaw']) assert.equal($(`apply-${a}`).disabled, true);
  assert.equal($('download').disabled, true);
  await context.dashboard.applyGains('yaw');
  assert.equal(requests.length, 2);
  console.log('Four-axis dashboard JavaScript smoke test passed');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
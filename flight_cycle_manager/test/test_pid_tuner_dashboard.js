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
    this.children = [];
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
                  'position-scale', 'yaw-scale', 'clear', 'scale-status', 'connection',
                  'transition-controls', 'transition-checks', 'transition-summary',
                  'apply-transition', 'reset-transition', 'transition-status', 'transition-reset']) {
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
    assert.ok(['/api/gains', '/api/transition'].includes(url));
    requests.push(JSON.parse(options.body));
    return new Promise(resolve => pending.push(resolve));
  },
  AbortSignal: {timeout: () => undefined},
  setTimeout: () => {},
});
vm.runInContext(script + `
globalThis.dashboard = {
  editors, applyGains, transitionEditor,
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
  assert.equal($('transition-controls').children.length, 10);
  assert.equal($('apply-transition').disabled, false);
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

  // Gate diagnostics show actual measured values and configured limits, not pose errors.
  const data = sample();
  data.state = 'DESCEND_CENTRAL';
  data.touchdown_transition = {
    evaluated: true, ready: false, fine_alignment_stable: false, height_ready: true,
    blockers: ['horizontal_speed'], checks: {
      horizontal_speed: {label: 'Measured horizontal speed', value: 0.12, limit: 0.07,
        unit: 'm/s', comparison: '<=', parameter: 'pid_touchdown_horizontal_speed_tolerance_m_s', passed: false},
      alignment_dwell: {label: 'Alignment dwell', value: 0, limit: 1, unit: 's',
        comparison: '>=', parameter: 'pid_alignment_stable_time_s', passed: false},
    },
    last_dwell_interruption: {time_s: 1, checks: {
      x: {label: '|X error|', value: 0.09, limit: 0.03, unit: 'm'},
    }},
  };
  context.dashboard.setTelemetry(data);
  assert.ok($('transition-summary').textContent.includes('fine_alignment_stable: false'));
  assert.ok($('transition-summary').textContent.includes('Measured horizontal speed'));
  assert.ok($('transition-reset').textContent.includes('0.090000 m; required <= 0.030000 m'));
  const rows = $('transition-checks').children;
  assert.ok(rows.some(row => row.children.some(cell => cell.textContent.includes('0.120000 m/s'))));

  // Dirty transition fields survive polling and independent gain edits.
  const field = $('transition-alignment_stable_time_s');
  field.value = '0.6';field.oninput();
  context.dashboard.setTelemetry(sample());
  assert.equal(field.value, '0.6');
  const transitionUpdate = $('apply-transition').onclick();
  assert.equal(requests.length, 3);
  assert.deepEqual(requests[2], {parameters: {pid_alignment_stable_time_s: 0.6}});
  assert.equal($('apply-transition').disabled, true);
  assert.equal($('apply-x').disabled, false);
  pending[2]({ok: false, json: async () => ({ok: false, message: 'transition rejected'})});
  await transitionUpdate;
  assert.equal(field.value, '0.6');
  assert.equal(context.dashboard.transitionEditor.dirty.size, 1);
  assert.equal($('transition-status').textContent, 'transition rejected');
  const retry = $('apply-transition').onclick();
  pending[3]({ok: true, json: async () => ({ok: true, message: 'parameters applied'})});
  await retry;
  assert.equal(context.dashboard.transitionEditor.dirty.size, 0);
  field.value = '0';field.oninput();
  await $('apply-transition').onclick();
  assert.equal(requests.length, 4);
  assert.equal($('transition-status').textContent, 'Use finite positive values');
  $('reset-transition').onclick();
  assert.equal(field.value, '0.1');
  context.dashboard.setTelemetry({...sample(), state: 'TOUCHDOWN_APPROACH'});
  assert.equal($('apply-transition').disabled, true);
  assert.equal(field.disabled, true);

  // Base-bank validation is local to the edited axis, and disconnect disables all updates.
  $('kp-z').value = '-1';
  $('kp-z').oninput();
  await $('apply-z').onclick();
  assert.equal(requests.length, 4);
  assert.ok($('gain-status-z').textContent.includes('Use nonnegative gains'));
  assert.equal($('gain-status-x').textContent, 'X applied');
  assert.equal($('download').disabled, false);
  context.dashboard.setTelemetry(null);
  for (const a of ['x', 'y', 'z', 'yaw']) assert.equal($(`apply-${a}`).disabled, true);
  assert.equal($('download').disabled, true);
  assert.equal($('apply-transition').disabled, true);
  assert.equal($('transition-checks').html, '');
  assert.equal($('transition-checks').children.length, 0);
  await context.dashboard.applyGains('yaw');
  assert.equal(requests.length, 4);
  console.log('Four-axis and transition dashboard JavaScript smoke test passed');
}
main().catch(error => { console.error(error); process.exitCode = 1; });
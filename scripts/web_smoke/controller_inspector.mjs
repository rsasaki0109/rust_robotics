// Inspect real deterministic traces; all changes use the rendered canvas UI.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => Object.fromEntries(
  Object.entries(c.dataset).filter(([key]) => key.startsWith('arena'))));
const settle = (page) => page.waitForTimeout(100);
const snapshot = (d) => Object.fromEntries(['arenaFrame', 'arenaCurrent', 'arenaSeries', 'arenaGoal'].map((key) => [key, d[key]]));
const receipts = new WeakMap();
const close = (actual, expected, message) => assert(Math.abs(actual - expected) < 8e-8, `${message}: ${actual} != ${expected}`);

function point(d, name, fraction) {
  if (fraction === undefined) return JSON.parse(d.arenaActions)[name];
  const [x, y, width, height] = JSON.parse(d.arenaPlots)[name === 'turn-plot' ? 1 : 0];
  return [x + width * fraction, y + height / 2];
}

async function aim(page, name, target, fraction) {
  // A layout revision can belong to hover, before a queued wheel is applied.
  // Process two real pointer moves, then require the actual widget hit test.
  // Never retry a tap: a delayed first tap could toggle a control twice.
  for (const dx of [-2, 0]) {
    const before = await state(page);
    // eframe reads MouseEvent.clientX/Y as integer CSS coordinates.
    const position = [Math.floor(target[0] + dx), Math.floor(target[1])];
    await page.mouse.move(...position);
    await page.waitForFunction(({ revision, position }) => {
      const d = document.querySelector('canvas')?.dataset;
      const pointer = JSON.parse(d?.arenaPointer ?? 'null');
      return Number(d?.arenaUiRevision) > revision && pointer && Math.hypot(pointer[0] - position[0], pointer[1] - position[1]) < 0.75;
    }, { revision: Number(before.arenaUiRevision), position });
    const after = await state(page);
    const current = point(after, name, fraction);
    if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) return false;
  }
  return JSON.parse((await state(page)).arenaHits)[name] === true;
}

async function scrollAction(page, name, fraction) {
  const before = await state(page);
  const previous = point(before, name, fraction)[1];
  // Leaving the drawing canvas can relayout between the scroll decision and
  // this processed hover. Recheck before asking a boundary to scroll further.
  if (previous >= 64 && previous <= page.viewportSize().height - 24) return;
  const delta = previous < 64 ? -280 : 280;
  await page.mouse.wheel(0, delta);
  await page.waitForFunction(({ revision, name, fraction, previous, delta }) => {
    const d = document.querySelector('canvas')?.dataset;
    if (!d || Number(d.arenaUiRevision) <= revision) return false;
    const y = fraction === undefined ? JSON.parse(d.arenaActions)[name][1]
      : (() => { const r = JSON.parse(d.arenaPlots)[name === 'turn-plot' ? 1 : 0]; return r[1] + r[3] / 2; })();
    return delta < 0 ? y > previous + 1 : y < previous - 1;
  }, { revision: Number(before.arenaUiRevision), name, fraction, previous, delta });
}

async function action(page, name, touch, fraction) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const target = point(await state(page), name, fraction);
    assert(target, `missing arena action: ${name}`);
    const { width, height } = page.viewportSize();
    if (target[0] >= 0 && target[0] <= width && target[1] >= 64 && target[1] <= height - 24) {
      await settle(page);
      if (!await aim(page, name, target, fraction)) continue;
      const before = await state(page);
      const current = point(before, name, fraction);
      if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) continue;
      if (!JSON.parse(before.arenaHits)[name]) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      await page.waitForFunction((count) => Number(document.querySelector('canvas')?.dataset.arenaActionCount) > count, Number(before.arenaActionCount));
      const after = await state(page);
      assert.equal(after.arenaLastAction, name, `input hit a different control than ${name}`);
      assert.equal(Number(after.arenaActionCount), Number(before.arenaActionCount) + 1, `${name} was handled more than once`);
      receipts.get(page)?.push({ name, count: Number(after.arenaActionCount), frame: Number(after.arenaFrame), playing: after.arenaPlaying,
        position: current, pointer: JSON.parse(before.arenaPointer), hit: JSON.parse(before.arenaHits)[name] });
      return after;
    }
    await page.mouse.move(Math.min(width - 20, Math.max(20, target[0])), height - 60);
    await scrollAction(page, name, fraction);
    await settle(page);
  }
  throw Error(`arena action is unreachable: ${name}`);
}

export async function checkControllerScroll(page, shots) {
  receipts.set(page, []);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.arenaReady === '1');
  const initial = await state(page);
  assert.equal(initial.arenaFrame, '100');
  checkMetrics(initial);
  for (const [name, fraction] of [['lqr', 0.6], ['pp', 0.25], ['stanley', 0.6]]) {
    const before = await state(page);
    const selected = await action(page, name, true);
    assert.equal(selected.arenaFocus, name);
    assert.deepEqual(snapshot(selected), snapshot(before));
    const inspected = await action(page, 'error-plot', true, fraction);
    assert.equal(Number(inspected.arenaFrame), Math.round(Number(inspected.arenaTotal) * fraction));
    checkMetrics(inspected);
  }
  const times = await page.evaluate(() => window.__slowPaintTimes);
  assert(times.length >= 3 && times.slice(1).every((time, i) => time - times[i] >= 350), 'scroll regression did not use slow paints');
  if (shots) await writeFile(join(shots, 'controller-inspector-scroll.json'), JSON.stringify({ paintTimes: times, inputs: receipts.get(page), inspected: await state(page) }, null, 2));
}

async function mapVisible(page) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const rect = JSON.parse((await state(page)).arenaRect);
    if (rect[1] >= 48 && rect[1] + rect[3] <= page.viewportSize().height) return rect;
    await page.mouse.move(page.viewportSize().width / 2, 120);
    const revision = Number((await state(page)).arenaUiRevision);
    await page.mouse.wheel(0, -300);
    await page.waitForFunction((previous) => Number(document.querySelector('canvas')?.dataset.arenaUiRevision) > previous, revision);
    await settle(page);
  }
  throw Error('arena map is unreachable');
}

function checkMetrics(d) {
  assert.equal(d.arenaPlaying, '0');
  const current = JSON.parse(d.arenaCurrent);
  const series = JSON.parse(d.arenaSeries);
  const goal = JSON.parse(d.arenaGoal);
  assert.deepEqual(current.map((row) => row.id), ['pp', 'stanley', 'lqr']);
  current.forEach((row, index) => {
    const prefix = series[index];
    assert.equal(prefix.length, row.frame + 1, 'a graph includes unobserved samples');
    assert(row.frame <= Number(d.arenaFrame));
    close(row.time, row.frame * 0.1, 'selected sample time');
    prefix.forEach(([time, omega, error], i) => {
      close(time, i * 0.1, 'graph sample time');
      assert(Math.abs(omega) <= 1.4 && error >= 0);
    });
    close(row.omega, prefix.at(-1)[1], 'applied command');
    close(row.error, prefix.at(-1)[2], 'instantaneous path error');
    close(row.rmse, Math.sqrt(prefix.reduce((sum, p) => sum + p[2] ** 2, 0) / prefix.length), 'prefix RMSE');
    close(row.max, Math.max(...prefix.map((p) => p[2])), 'prefix maximum error');
    const changes = prefix.slice(1).map((p, i) => (p[1] - prefix[i][1]) ** 2);
    close(row.smooth, Math.sqrt(changes.reduce((a, b) => a + b, 0) / Math.max(1, changes.length)), 'prefix turn change');
    close(row.goal, Math.hypot(row.x - goal[0], row.y - goal[1]), 'distance to the goal now');
  });
}

async function shareWithVisitor(page, base, d, touch) {
  const context = await page.context().browser().newContext({ viewport: page.viewportSize(), hasTouch: touch, deviceScaleFactor: touch ? 2 : 1 });
  try {
    const other = await context.newPage();
    const errors = [];
    other.on('pageerror', (error) => errors.push(error.message));
    await other.goto(`${base}?${d.arenaShare}`, { waitUntil: 'load' });
    await other.waitForFunction(() => document.querySelector('canvas')?.dataset.arenaReady === '1');
    const shared = await state(other);
    assert.equal(shared.arenaShare, d.arenaShare);
    assert.equal(shared.arenaFocus, d.arenaFocus);
    assert.deepEqual(snapshot(shared), snapshot(d), 'the shared moment changed the actual trace or metrics');
    checkMetrics(shared);
    assert.deepEqual(errors, []);
    return shared;
  } finally { await context.close(); }
}

export async function checkControllerInspector(page, base, shots, touch) {
  receipts.set(page, []);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.arenaReady === '1');
  const initial = await state(page);
  assert.equal(initial.arenaFrame, '100');
  assert.equal(initial.arenaFocus, 'stanley');
  checkMetrics(initial);
  await page.waitForTimeout(300);
  assert.deepEqual(snapshot(await state(page)), snapshot(initial), 'a paused trace advanced');
  const previous = await action(page, 'back', touch);
  assert.equal(previous.arenaFrame, '99');
  const next = await action(page, 'next', touch);
  assert.deepEqual(snapshot(next), snapshot(initial));
  const before = JSON.parse(previous.arenaCurrent);
  JSON.parse(next.arenaCurrent).forEach((row, i) => {
    close(row.x, before[i].x + row.v * Math.cos(before[i].yaw) * 0.1, 'command / position synchronization');
    close(row.y, before[i].y + row.v * Math.sin(before[i].yaw) * 0.1, 'command / position synchronization');
    close(Math.atan2(Math.sin(row.yaw - before[i].yaw), Math.cos(row.yaw - before[i].yaw)), row.omega * 0.1, 'command / heading synchronization');
  });
  const focused = await action(page, 'lqr', touch);
  assert.equal(focused.arenaFocus, 'lqr');
  assert.deepEqual(snapshot(focused), snapshot(initial), 'selecting a controller altered the simulation');
  const selected = await action(page, 'error-plot', touch, 0.6);
  assert.equal(Number(selected.arenaFrame), Math.round(Number(selected.arenaTotal) * 0.6));
  checkMetrics(selected);
  await action(page, 'slow', touch);
  const sharedMoment = await state(page);
  const shared = await shareWithVisitor(page, base, sharedMoment, touch);
  await mapVisible(page);
  if (shots) await page.screenshot({ path: join(shots, touch ? 'controller-inspector-moment-phone.png' : 'controller-inspector-moment.png') });
  if (touch) {
    await page.setViewportSize({ width: 844, height: 390 });
    await settle(page);
    assert.deepEqual(snapshot(await state(page)), snapshot(sharedMoment), 'rotation advanced a paused trace');
    await page.setViewportSize({ width: 320, height: 740 });
    await settle(page);
  }
  const turnSeek = await action(page, 'turn-plot', touch, 0.25);
  assert.equal(Number(turnSeek.arenaFrame), Math.round(Number(turnSeek.arenaTotal) * 0.25));
  checkMetrics(turnSeek);
  const end = await action(page, 'end', touch);
  assert.equal(end.arenaFrame, end.arenaTotal);
  assert(JSON.parse(end.arenaCurrent).every((row) => row.ended));
  checkMetrics(end);
  const start = await action(page, 'start', touch);
  assert.equal(start.arenaFrame, '0');
  assert(JSON.parse(start.arenaSeries).every((series) => series.length === 1 && series[0][1] === 0));
  checkMetrics(start);
  await action(page, 'fast', touch);
  await action(page, 'play', touch);
  await page.waitForFunction(() => Number(document.querySelector('canvas')?.dataset.arenaFrame) >= 6);
  const paused = await action(page, 'play', touch);
  assert.equal(paused.arenaPlaying, '0');
  checkMetrics(paused);
  await page.waitForTimeout(300);
  assert.deepEqual(snapshot(await state(page)), snapshot(paused));
  await action(page, 'weak', touch);
  const weak = await action(page, 'end', touch);
  assert.equal(weak.arenaResponse, '0.5');
  checkMetrics(weak);
  await action(page, 'ideal', touch);
  const ideal = await action(page, 'end', touch);
  assert.equal(ideal.arenaResponse, '1');
  assert.notEqual(weak.arenaSeries, ideal.arenaSeries, 'response edits retained stale commands');
  checkMetrics(ideal);

  await action(page, 'draw', touch);
  assert.equal((await state(page)).arenaDrawing, '1');
  const [x, y, width, height] = await mapVisible(page);
  const strokePoint = (i) => [x + width * (0.08 + i * 0.02), y + height * (0.5 + 0.15 * Math.cos(i / 7))];
  if (touch) {
    const cdp = await page.context().newCDPSession(page);
    const finger = (i) => { const [x, y] = strokePoint(i); return { id: 1, x, y, radiusX: 7, radiusY: 7, force: 1 }; };
    try {
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [finger(0)] });
      for (let i = 1; i <= 40; i++) {
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [finger(i)] });
        await page.waitForTimeout(30);
      }
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    } finally { await cdp.detach(); }
  } else {
    await page.mouse.move(...strokePoint(0));
    await page.mouse.down();
    for (let i = 1; i <= 40; i++) {
      await page.mouse.move(...strokePoint(i));
      await page.waitForTimeout(30);
    }
    await page.mouse.up();
  }
  await page.waitForFunction(() => {
    const d = document.querySelector('canvas')?.dataset;
    return d?.arenaCustom === '1' && d.arenaDrawing === '0' && d.arenaReady === '1';
  });
  const custom = await action(page, 'end', touch);
  assert(new URLSearchParams(custom.arenaShare).has('course'));
  checkMetrics(custom);
  await shareWithVisitor(page, base, custom, touch);
  await action(page, 'reset', touch);
  const reset = await action(page, 'start', touch);
  assert.equal(reset.arenaCustom, '0');
  assert.equal(reset.arenaPreset, 'slalom');
  assert.equal(reset.arenaResponse, '0.85');
  assert.equal(reset.arenaFocus, 'pp');
  assert.equal(reset.arenaFrame, '0');
  checkMetrics(reset);
  if (shots) await writeFile(join(shots, touch ? 'controller-inspector-phone.json' : 'controller-inspector.json'), JSON.stringify({ initial, selected: sharedMoment, shared, end, weak, ideal, custom, reset, inputs: receipts.get(page) }, null, 2));
}

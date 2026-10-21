// Read derived diagnostics, but drive every change through the real canvas UI.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => Object.fromEntries(
  Object.entries(c.dataset).filter(([key]) => key.startsWith('loc'))));
const settle = (page) => page.waitForTimeout(100);
const inputReceipts = new WeakMap();

async function aim(page, name, target) {
  // A hover paint can precede a queued wheel. Require two processed pointer
  // moves and the widget's actual hit test before sending one real tap.
  for (const dx of [-2, 0]) {
    const before = await state(page);
    const position = [Math.floor(target[0] + dx), Math.floor(target[1])];
    await page.mouse.move(...position);
    await page.waitForFunction(({ revision, position }) => {
      const d = document.querySelector('canvas')?.dataset;
      const pointer = JSON.parse(d?.locPointer ?? 'null');
      return Number(d?.locUiRevision) > revision && d.locScrolling === '0' && pointer && Math.hypot(pointer[0] - position[0], pointer[1] - position[1]) < 0.75;
    }, { revision: Number(before.locUiRevision), position });
    const current = JSON.parse((await state(page)).locActions)[name];
    if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) return false;
  }
  return JSON.parse((await state(page)).locHits)[name] === true;
}

async function action(page, name, touch) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const point = JSON.parse((await state(page)).locActions)[name];
    assert(point, `missing localization action: ${name}`);
    const { width, height } = page.viewportSize();
    if (point[1] >= 64 && point[1] <= height - 24) {
      if (!await aim(page, name, point)) continue;
      const before = await state(page);
      const current = JSON.parse(before.locActions)[name];
      if (Math.abs(current[0] - point[0]) > 1 || Math.abs(current[1] - point[1]) > 1) continue;
      if (!JSON.parse(before.locHits)[name]) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      // Touch release can be queued until the next egui frame. Wait for the
      // actual response, never assume a fixed delay means the input was handled.
      await page.waitForFunction((count) => Number(document.querySelector('canvas')?.dataset.locActionCount) > count, Number(before.locActionCount));
      const after = await state(page);
      assert.equal(after.locLastAction, name, `input hit a different control than ${name}`);
      assert.equal(Number(after.locActionCount), Number(before.locActionCount) + 1, `${name} was handled more than once`);
      inputReceipts.get(page)?.push({ name, count: Number(after.locActionCount), layout: Number(after.locUiRevision), playing: after.locPlaying, steps: Number(after.locSteps), position: current, pointer: JSON.parse(before.locPointer), hit: JSON.parse(before.locHits)[name], scrolling: before.locScrolling === '1', scrollRevision: Number(before.locScrollRevision) });
      return;
    }
    await page.mouse.move(Math.min(width - 20, Math.max(20, point[0])), height - 60);
    const before = await state(page), previous = JSON.parse(before.locActions)[name][1];
    if (previous >= 64 && previous <= height - 24) continue;
    const delta = previous < 64 ? -280 : 280;
    await page.mouse.wheel(0, delta);
    await page.waitForFunction(({ revision, scrollRevision, name, previous, delta }) => {
      const d = document.querySelector('canvas')?.dataset;
      if (!d || Number(d.locUiRevision) <= revision || Number(d.locScrollRevision) <= scrollRevision || d.locScrolling !== '0') return false;
      const y = JSON.parse(d.locActions)[name][1];
      return delta < 0 ? y > previous + 1 : y < previous - 1;
    }, { revision: Number(before.locUiRevision), scrollRevision: Number(before.locScrollRevision), name, previous, delta });
  }
  throw Error(`localization action is unreachable: ${name}`);
}

async function mapVisible(page) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const rect = JSON.parse((await state(page)).locRect);
    if (rect[1] >= 48 && rect[1] + rect[3] <= page.viewportSize().height) return rect;
    await page.mouse.move(page.viewportSize().width / 2, 120);
    const before = await state(page), previous = JSON.parse(before.locRect)[1];
    await page.mouse.wheel(0, -300);
    await page.waitForFunction(({ revision, scrollRevision, previous }) => {
      const d = document.querySelector('canvas')?.dataset;
      return d && Number(d.locUiRevision) > revision && Number(d.locScrollRevision) > scrollRevision && d.locScrolling === '0' && JSON.parse(d.locRect)[1] > previous + 1;
    }, { revision: Number(before.locUiRevision), scrollRevision: Number(before.locScrollRevision), previous });
  }
  throw Error('localization map is unreachable');
}

const sample = (d) => Object.fromEntries(['locSteps', 'locTruth', 'locEstimate', 'locRanges', 'locFix', 'locErrors'].map((key) => [key, d[key]]));

function checkError(d) {
  const [x, y] = JSON.parse(d.locTruth);
  const [ex, ey] = JSON.parse(d.locEstimate);
  const errors = JSON.parse(d.locErrors);
  assert(errors.length > 0 && errors.length <= 120);
  assert(Math.abs(errors.at(-1) - Math.hypot(x - ex, y - ey)) < 3e-8,
    'the graph does not report the actual estimate error');
}

export async function checkLocalization(page, base, shots, touch, slowPaint = false) {
  inputReceipts.set(page, []);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.locPlaying === '0');
  const initial = await state(page);
  assert.equal(initial.locSteps, '0');
  assert.equal(initial.locFilter, 'pf');
  assert.equal(initial.locTour, '0');
  assert.deepEqual(JSON.parse(initial.locRanges), []);
  await action(page, 'step', touch);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.locSteps === '1');
  const stationary = await state(page);
  assert.equal(stationary.locTruth, initial.locTruth, 'stationary sensing moved the true robot');
  assert.equal(JSON.parse(stationary.locRanges).length, 5);
  assert.equal(stationary.locFix, 'null');
  checkError(stationary);
  await page.keyboard.down('ArrowUp');
  await page.waitForTimeout(300);
  await page.keyboard.up('ArrowUp');
  assert.deepEqual(sample(await state(page)), sample(stationary), 'pause allowed motion or new observations');

  await action(page, 'quiet', touch);
  assert.equal((await state(page)).locNoise, '0.3');
  await action(page, 'noisy', touch);
  assert.equal((await state(page)).locNoise, '3');
  assert.deepEqual(sample(await state(page)), sample(stationary), 'noise changed an already captured observation');
  await action(page, 'observations', touch);
  assert.equal((await state(page)).locObservations, '0');
  assert.deepEqual(sample(await state(page)), sample(stationary));
  await action(page, 'observations', touch);
  await action(page, 'anchor-d', touch);
  assert.equal(new URLSearchParams((await state(page)).locShare).get('loc_anchor'), '3');
  await action(page, 'tour', touch);
  await action(page, 'play', touch);
  await page.waitForFunction(() => Number(document.querySelector('canvas')?.dataset.locSteps) >= 22);
  await action(page, 'play', touch);
  const pf = await state(page);
  assert.equal(pf.locPlaying, '0');
  assert.notEqual(pf.locTruth, initial.locTruth);
  checkError(pf);
  await page.waitForTimeout(300);
  assert.deepEqual(sample(await state(page)), sample(pf));
  const [x, y, width, height] = await mapVisible(page);
  if (shots) {
    await page.screenshot({ path: join(shots, touch ? 'localization-pf-phone.png' : 'localization-pf.png') });
    if (!touch) await page.screenshot({ path: join(shots, 'localization-map.png'), clip: { x, y, width, height } });
  }

  // Shares deliberately start a new random run, preserving settings, not a false replay.
  const visitor = await page.context().browser().newContext({
    viewport: page.viewportSize(), hasTouch: touch, deviceScaleFactor: touch ? 2 : 1,
  });
  let shared;
  try {
    const other = await visitor.newPage();
    const problems = [];
    other.on('pageerror', (error) => problems.push(error.message));
    await other.goto(`${base}?${pf.locShare}`, { waitUntil: 'load' });
    await other.waitForFunction(() => document.querySelector('canvas')?.dataset.locPlaying === '0');
    shared = await state(other);
    assert.equal(shared.locSteps, '0');
    assert.equal(shared.locShare, pf.locShare);
    assert.equal(shared.locRanges, '[]');
    assert.deepEqual(problems, []);
  } finally { await visitor.close(); }

  if (touch) {
    await page.setViewportSize({ width: 844, height: 390 });
    await page.waitForTimeout(250);
    assert.deepEqual(sample(await state(page)), sample(pf), 'rotation advanced paused observations');
    await mapVisible(page);
    if (shots) await page.screenshot({ path: join(shots, 'localization-landscape.png') });
    await page.setViewportSize({ width: 320, height: 740 });
    await settle(page);
  }
  await action(page, 'ekf', touch);
  const reset = await state(page);
  assert.equal(reset.locFilter, 'ekf');
  assert.equal(reset.locSteps, '0', 'switching filter retained a stale estimate');
  assert.equal(reset.locPlaying, '0');
  assert.equal(reset.locNoise, '3');
  assert.equal(reset.locRanges, '[]');
  assert.equal(reset.locFix, 'null');
  await action(page, 'step', touch);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.locSteps === '1');
  const ekf = await state(page);
  assert.equal(ekf.locRanges, '[]');
  assert.equal(JSON.parse(ekf.locFix).length, 2);
  checkError(ekf);
  await mapVisible(page);
  if (shots) await page.screenshot({ path: join(shots, touch ? 'localization-ekf-phone.png' : 'localization-ekf.png') });
  await action(page, 'reset', touch);
  const cleared = await state(page);
  assert.equal(cleared.locSteps, '0');
  assert.equal(cleared.locFix, 'null');
  assert.equal(cleared.locErrors, '[]');
  assert.equal(cleared.locPlaying, '0');
  const receipts = inputReceipts.get(page);
  assert.equal(receipts.length, 12);
  assert.deepEqual(receipts.map((r) => r.count), Array.from({ length: 12 }, (_, i) => i + 1));
  assert(receipts.every((r) => r.hit), 'localization tapped a control without a processed hit test');
  assert(receipts.every((r) => !r.scrolling), 'localization tapped while smooth scrolling was active');
  if (shots && !slowPaint) await writeFile(join(shots, `localization-input-sync${touch ? '-phone' : '-desktop'}.json`), JSON.stringify({
    inputs: receipts, paused: sample(pf), cleared: sample(cleared),
  }, null, 2));
  if (slowPaint) {
    const times = await page.evaluate(() => window.__slowPaintTimes);
    assert(times.length >= 3 && times.slice(1).every((time, i) => time - times[i] >= 350),
      'localization input regression did not actually use slow paints');
    if (shots) await writeFile(join(shots, 'localization-input-sync.json'), JSON.stringify({
      paintTimes: times, inputs: inputReceipts.get(page), paused: sample(pf), cleared: sample(cleared),
    }, null, 2));
  }
  if (shots && !touch) await writeFile(join(shots, 'localization-observations.json'), JSON.stringify(
    Object.fromEntries(Object.entries({ stationary, pf, shared, ekf, cleared }).map(([key, d]) => [key, Object.fromEntries(
      Object.entries(d).filter(([field]) => !['locActions', 'locRect'].includes(field)))])), null, 2));
}

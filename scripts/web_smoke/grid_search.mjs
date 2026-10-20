// Only real canvas buttons, pointer edits and shared URLs; never write planner state.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => ({ ...c.dataset }));
const wait = (page, predicate, arg) => page.waitForFunction(predicate, arg, { timeout: 15000 });
const settle = (page) => page.waitForTimeout(100);
const receipts = new WeakMap();

function point(d, name, fraction) {
  if (fraction === undefined) return JSON.parse(d.gridActions)[name];
  const [x, y, width, height] = JSON.parse(d.gridRect);
  return [x + width * fraction[0], y + height * fraction[1]];
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
    await wait(page, ({ revision, position }) => {
      const d = document.querySelector('canvas')?.dataset;
      const pointer = JSON.parse(d?.gridPointer ?? 'null');
      return Number(d?.gridUiRevision) > revision && pointer && Math.hypot(pointer[0] - position[0], pointer[1] - position[1]) < 0.75;
    }, { revision: Number(before.gridUiRevision), position });
    const after = await state(page);
    const current = point(after, name, fraction);
    if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) return false;
  }
  return JSON.parse((await state(page)).gridHits)[name] === true;
}

async function scrollAction(page, name, fraction) {
  const before = await state(page);
  const previous = point(before, name, fraction)[1];
  // Rotation and replay controls can relayout before this processed hover.
  // Recheck before asking a scroll boundary to move further.
  if (previous >= 64 && previous <= page.viewportSize().height - 24) return;
  const delta = previous < 64 ? -280 : 280;
  await page.mouse.wheel(0, delta);
  await wait(page, ({ revision, name, fraction, previous, delta }) => {
    const d = document.querySelector('canvas')?.dataset;
    if (!d || Number(d.gridUiRevision) <= revision) return false;
    const y = fraction === undefined ? JSON.parse(d.gridActions)[name][1]
      : (() => { const r = JSON.parse(d.gridRect); return r[1] + r[3] * fraction[1]; })();
    return delta < 0 ? y > previous + 1 : y < previous - 1;
  }, { revision: Number(before.gridUiRevision), name, fraction, previous, delta });
}

async function action(page, name, touch, fraction) {
  for (let attempt = 0; attempt < 12; attempt++) {
    const target = point(await state(page), name, fraction);
    assert(target, `missing grid action: ${name}`);
    const { width, height } = page.viewportSize();
    if (target[0] >= 0 && target[0] <= width && target[1] >= 64 && target[1] <= height - 24) {
      await settle(page);
      if (!await aim(page, name, target, fraction)) continue;
      const before = await state(page);
      const current = point(before, name, fraction);
      if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) continue;
      if (!JSON.parse(before.gridHits)[name]) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      await wait(page, (count) => Number(document.querySelector('canvas')?.dataset.gridActionCount) > count, Number(before.gridActionCount));
      const after = await state(page);
      assert.equal(after.gridLastAction, name, `input hit a different control than ${name}`);
      assert.equal(Number(after.gridActionCount), Number(before.gridActionCount) + 1, `${name} was handled more than once`);
      receipts.get(page)?.push({ name, count: Number(after.gridActionCount), step: Number(after.gridSearchStep), playing: after.gridPlaying, found: after.gridFound, planner: after.gridPlanner,
        position: current, pointer: JSON.parse(before.gridPointer), hit: JSON.parse(before.gridHits)[name] });
      return after;
    }
    await page.mouse.move(Math.min(width - 20, Math.max(20, target[0])), height - 60);
    await scrollAction(page, name, fraction);
    await settle(page);
  }
  throw Error(`grid action is unreachable: ${name}`);
}

async function gridVisible(page) {
  const fits = (r) => r[1] >= 48 && r[1] + r[3] <= page.viewportSize().height;
  for (let attempt = 0; attempt < 12; attempt++) {
    const d = await state(page);
    const rect = JSON.parse(d.gridRect);
    if (fits(rect)) {
      if (!await aim(page, 'map', point(d, 'map', [0.5, 0.5]), [0.5, 0.5])) continue;
      const current = JSON.parse((await state(page)).gridRect);
      if (fits(current)) return current;
      continue;
    }
    await page.mouse.move(page.viewportSize().width / 2, 120);
    const before = await state(page);
    if (fits(JSON.parse(before.gridRect))) continue;
    const previous = JSON.parse(before.gridRect)[1];
    await page.mouse.wheel(0, -300);
    await wait(page, ({ revision, previous }) => {
      const d = document.querySelector('canvas')?.dataset;
      return Number(d?.gridUiRevision) > revision && JSON.parse(d.gridRect)[1] > previous + 1;
    }, { revision: Number(before.gridUiRevision), previous });
    await settle(page);
  }
  throw Error('the grid could not be reached by scrolling');
}

export async function checkGridSearch(page, base, shots, touch) {
  receipts.set(page, []);
  await wait(page, () => document.querySelector('canvas')?.dataset.gridPlanner === 'astar');
  const initial = await state(page);
  assert.equal(initial.gridFound, '1');
  assert(initial.gridPath.startsWith('2,12;') && initial.gridPath.endsWith(';29,12'));
  await action(page, 'replay', touch);
  await wait(page, () => {
    const d = document.querySelector('canvas')?.dataset;
    return d?.gridPlaying === '1' && Number(d.gridSearchStep) > 0;
  });
  await action(page, 'pause', touch);
  const paused = await state(page);
  assert.equal(paused.gridPlaying, '0');
  assert.equal(paused.gridFound, 'pending');
  assert.equal(paused.gridPath, '', 'a future path leaked into the search replay');
  await page.waitForTimeout(250);
  assert.equal((await state(page)).gridExpanded, paused.gridExpanded, 'paused search continued to expand');
  await action(page, 'next', touch);
  const moment = await state(page);
  assert.equal(Number(moment.gridSearchStep), Number(paused.gridSearchStep) + 1);
  assert.equal(moment.gridExpanded.split(';').length, Number(moment.gridSearchStep));
  if (shots) await page.screenshot({ path: join(shots, touch ? 'grid-search-touch-paused.png' : 'grid-search-paused.png') });

  const visitor = await page.context().browser().newContext({
    viewport: page.viewportSize(), hasTouch: touch, deviceScaleFactor: touch ? 2 : 1,
  });
  let shared;
  try {
    const other = await visitor.newPage();
    const errors = [];
    other.on('pageerror', (error) => errors.push(error.message));
    await other.goto(`${base}?${moment.gridShare}`, { waitUntil: 'load' });
    await wait(other, (step) => document.querySelector('canvas')?.dataset.gridSearchStep === step, moment.gridSearchStep);
    shared = await state(other);
    for (const field of ['gridPlanner', 'gridSearchStep', 'gridSearchTotal', 'gridPlaying', 'gridFound', 'gridExpanded', 'gridPath']) {
      assert.equal(shared[field], moment[field], `shared ${field} differs`);
    }
    await other.waitForTimeout(250);
    assert.equal((await state(other)).gridExpanded, moment.gridExpanded);
    assert.deepEqual(errors, []);
  } finally { await visitor.close(); }

  if (touch) {
    await page.setViewportSize({ width: 844, height: 390 });
    await page.waitForTimeout(250);
    assert.equal((await state(page)).gridExpanded, moment.gridExpanded);
    await gridVisible(page);
    if (shots) await page.screenshot({ path: join(shots, 'grid-search-landscape.png') });
    await page.setViewportSize({ width: 320, height: 740 });
    await settle(page);
  }
  await action(page, 'result', touch);
  assert.equal((await state(page)).gridPath, initial.gridPath);
  await action(page, 'visible', touch);
  assert.equal((await state(page)).gridExpanded, '');
  assert.equal((await state(page)).gridPath, initial.gridPath);
  await action(page, 'visible', touch);
  assert((await state(page)).gridExpanded.length > 0);
  await action(page, 'dijkstra', touch);
  await wait(page, () => document.querySelector('canvas')?.dataset.gridPlanner === 'dijkstra');
  const dijkstra = await state(page);
  assert.equal(dijkstra.gridFound, '1', 'Dijkstra rejected the same one-cell gap');
  assert(Number(dijkstra.gridSearchTotal) > Number(initial.gridSearchTotal));
  if (shots) {
    const [x, y, width, height] = await gridVisible(page);
    await page.screenshot({ path: join(shots, touch ? 'grid-dijkstra-phone.png' : 'grid-dijkstra.png') });
    if (!touch) await page.screenshot({ path: join(shots, 'grid-search-map.png'), clip: { x, y, width, height } });
  }

  await action(page, 'blocked', touch);
  const blocked = await state(page);
  assert.equal(blocked.gridFound, '0');
  assert.equal(blocked.gridPath, '');
  assert(Number(blocked.gridSearchTotal) > 0);
  assert(blocked.gridExpanded.split(';').every((cell) => Number(cell.split(',')[0]) < 15));
  await gridVisible(page);
  if (shots) await page.screenshot({ path: join(shots, touch ? 'grid-blocked-phone.png' : 'grid-blocked.png') });
  // Open exactly one cell in the separating wall, by actual mouse/touch editing.
  await gridVisible(page);
  await action(page, 'map', touch, [15.5 / 32, 12.5 / 24]);
  await wait(page, () => document.querySelector('canvas')?.dataset.gridFound === '1');
  const opened = await state(page);
  assert.equal(opened.gridPlaying, '0');
  assert(opened.gridPath.includes('15,12'), 'the new route did not use the opened gap');
  assert.equal(opened.gridSearchStep, opened.gridSearchTotal, 'editing retained an old replay cursor');
  await action(page, 'detour', touch);
  const detour = await state(page);
  assert.equal(detour.gridFound, '1');
  assert(detour.gridPath.includes('15,5'), 'the detour ignored its only gap');
  if (shots && !touch) {
    const [x, y, width, height] = await gridVisible(page);
    await page.screenshot({ path: join(shots, 'grid-search-map.png'), clip: { x, y, width, height } });
    const pick = (d) => Object.fromEntries(['gridPlanner', 'gridSearchStep', 'gridSearchTotal', 'gridPlaying', 'gridFound', 'gridPath', 'gridExpanded', 'gridShare'].map((key) => [key, d[key]]));
    await writeFile(join(shots, 'grid-search-replay.json'), JSON.stringify(Object.fromEntries(
      Object.entries({ initial, moment, shared, dijkstra, blocked, opened, detour }).map(([key, d]) => [key, pick(d)])), null, 2));
  }
}

export async function checkGridScroll(page, shots) {
  receipts.set(page, []);
  await wait(page, () => document.querySelector('canvas')?.dataset.gridPlanner === 'astar');
  await action(page, 'dijkstra', true);
  assert.equal((await state(page)).gridFound, '1');
  await gridVisible(page);
  const blocked = await action(page, 'blocked', true);
  assert.equal(blocked.gridFound, '0');
  assert.equal(blocked.gridPath, '');
  assert(Number(blocked.gridSearchTotal) > 0);
  assert(blocked.gridExpanded.split(';').every((cell) => Number(cell.split(',')[0]) < 15));
  await gridVisible(page);
  const detour = await action(page, 'detour', true);
  assert.equal(detour.gridFound, '1');
  assert(detour.gridPath.includes('15,5'));
  await gridVisible(page);
  assert.equal((await action(page, 'blocked', true)).gridFound, '0');
  const reset = await action(page, 'reset', true);
  assert.equal(reset.gridPlanner, 'astar');
  assert.equal(reset.gridFound, '1');
  assert(reset.gridPath.includes('15,12'));
  assert.deepEqual(receipts.get(page).map((r) => r.count), [1, 2, 3, 4, 5]);
  const times = await page.evaluate(() => window.__slowPaintTimes);
  assert(times.length >= 3 && times.slice(1).every((time, i) => time - times[i] >= 350), 'grid input regression did not use slow paints');
  if (shots) await writeFile(join(shots, 'grid-input-sync.json'), JSON.stringify({ paintTimes: times, inputs: receipts.get(page), blocked, detour, reset }, null, 2));
}

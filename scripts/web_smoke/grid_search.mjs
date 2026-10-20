// Only real canvas buttons, pointer edits and shared URLs; never write planner state.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => ({ ...c.dataset }));
const wait = (page, predicate, arg) => page.waitForFunction(predicate, arg, { timeout: 15000 });
const settle = (page) => page.waitForTimeout(100);

async function action(page, name, touch) {
  for (let attempt = 0; attempt < 12; attempt++) {
    const d = await state(page);
    const point = JSON.parse(d.gridActions)[name];
    assert(point, `missing ${name} action`);
    const { width, height } = page.viewportSize();
    if (point[1] >= 64 && point[1] <= height - 24) {
      await settle(page);
      const current = JSON.parse((await state(page)).gridActions)[name];
      if (Math.abs(current[0] - point[0]) > 1 || Math.abs(current[1] - point[1]) > 1) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      await settle(page);
      return;
    }
    await page.mouse.move(width / 2, height - 60);
    await page.mouse.wheel(0, point[1] < 64 ? -280 : 280);
    await settle(page);
  }
  throw Error(`${name} could not be reached by scrolling`);
}

async function gridVisible(page) {
  for (let attempt = 0; attempt < 12; attempt++) {
    const [x, y, w, h] = JSON.parse((await state(page)).gridRect);
    if (y >= 48 && y + h <= page.viewportSize().height) return [x, y, w, h];
    await page.mouse.move(page.viewportSize().width / 2, 120);
    await page.mouse.wheel(0, -300);
    await settle(page);
  }
  throw Error('the grid could not be reached by scrolling');
}

export async function checkGridSearch(page, base, shots, touch) {
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
  const [x, y, w, h] = await gridVisible(page);
  const gap = [x + 15.5 * w / 32, y + 12.5 * h / 24];
  if (touch) await page.touchscreen.tap(...gap); else await page.mouse.click(...gap);
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

// Personal bests come from ordinary live flight, including the slow first lap.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { droneState, flyManualCourse, waitDrone } from './drone_manual.mjs';

const storageKey = 'rust-robotics.drone-personal-bests.v1';
const query = 'tab=drone&course=climb&mode=manual&cinema=1';
const stored = (page) => page.evaluate((key) => localStorage.getItem(key), storageKey);

async function resultAction(page, name) {
  const d = await droneState(page);
  const point = JSON.parse(d[`droneResult${name}`]);
  const viewport = page.viewportSize();
  assert(point[0] >= 50 && point[0] <= viewport.width - 50);
  assert(point[1] >= 20 && point[1] <= viewport.height - 20);
  await page.touchscreen.tap(...point);
}

export async function checkPersonalBest(page, base, shots) {
  assert.equal((await droneState(page)).droneBestSteps, '0');
  // Starting Play without held keys adds real hovering time, not fake states.
  const actions = JSON.parse((await droneState(page)).droneActions);
  await page.touchscreen.tap(...actions.pause);
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 80);
  const first = await flyManualCourse(page);
  assert.equal(first.droneNewBest, '1');
  assert.equal(first.droneBestSteps, first.droneFrame);
  assert.equal(first.droneBestPersistent, '1');
  const saved = await stored(page);
  assert(saved?.startsWith('v1\n'));
  if (shots) await page.screenshot({ path: `${shots}/drone-best-portrait.png` });
  await page.setViewportSize({ width: 844, height: 390 });
  await page.waitForTimeout(300);
  if (shots) await page.screenshot({ path: `${shots}/drone-best-landscape.png` });
  await resultAction(page, 'Best');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneReplay === '1');
  assert.equal((await droneState(page)).droneNewBest, '0');

  // Reopen the normal play entry. The ready-screen Replay button now watches
  // the saved best, including its real input history and exact final pose.
  await page.goto(`${base}?${query}`, { waitUntil: 'load' });
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.dronePad));
  let d = await droneState(page);
  assert.equal(d.droneFrame, '0');
  assert.equal(d.droneBestSteps, first.droneFrame);
  assert.equal(d.droneNewBest, '0');
  await page.touchscreen.tap(...JSON.parse(d.droneActions).replay);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished', 45000);
  d = await droneState(page);
  assert.equal(d.droneReplay, '1');
  assert.equal(d.droneNewBest, '0');
  assert.equal(d.dronePosition, first.dronePosition);
  assert.equal(d.droneFrame, first.droneFrame);
  assert.equal(await stored(page), saved, 'watching a best rewrote it');

  // A different visitor can watch this shared flight without claiming it as
  // their personal best. This context has a genuinely empty local store.
  const receiverContext = await page.context().browser().newContext({
    deviceScaleFactor: await page.evaluate(() => devicePixelRatio),
  });
  const receiver = await receiverContext.newPage();
  const problems = [];
  receiver.on('pageerror', (error) => problems.push(error.message));
  let shared;
  try {
    await receiver.goto(`${base}?${d.droneShare}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished' && document.querySelector('canvas')?.dataset.droneSeeking === '0');
    shared = await droneState(receiver);
    assert.equal(shared.droneReplay, '1');
    assert.equal(shared.droneBestSteps, '0');
    assert.equal(shared.droneNewBest, '0');
    assert.equal(shared.dronePosition, first.dronePosition);
    assert.equal(await stored(receiver), null);
    assert.deepEqual(problems, []);
  } finally { await receiverContext.close(); }
  await page.bringToFront();
  await resultAction(page, 'Again');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '0' && document.querySelector('canvas')?.dataset.droneReplay === '0');
  assert.equal((await droneState(page)).droneBestSteps, first.droneFrame);
  const faster = await flyManualCourse(page);
  assert(Number(faster.droneFrame) < Number(first.droneFrame), 'the second lap did not beat the deliberate hover');
  assert.equal(faster.droneNewBest, '1');
  assert.equal(faster.droneBestSteps, faster.droneFrame);
  assert.notEqual(await stored(page), saved);
  if (shots) {
    await page.screenshot({ path: `${shots}/drone-best-improved.png` });
    await writeFile(`${shots}/drone-best-flight.json`, JSON.stringify({ first, shared, faster }, null, 2) + '\n');
  }
}

export async function checkSessionBest(page, base, shots) {
  const finish = await flyManualCourse(page);
  assert.equal(finish.droneBestSteps, finish.droneFrame);
  assert.equal(finish.droneNewBest, '1');
  assert.equal(finish.droneBestPersistent, '0');
  if (shots) await page.screenshot({ path: `${shots}/drone-best-session.png` });
  await resultAction(page, 'Again');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '0');
  assert.equal((await droneState(page)).droneBestSteps, finish.droneFrame);
  await page.goto(`${base}?${query}`, { waitUntil: 'load' });
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.dronePad));
  assert.equal((await droneState(page)).droneBestSteps, '0');
}

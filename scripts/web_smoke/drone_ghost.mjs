// Race a best earned by a real flight, then share both aircraft with a visitor
// whose browser has never flown this course. No simulation states are injected.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { droneState, flyManualCourse, waitDrone } from './drone_manual.mjs';

const storageKey = 'rust-robotics.drone-personal-bests.v1';
const settle = (page) => page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));

async function retry(page) {
  await page.touchscreen.tap(...JSON.parse((await droneState(page)).droneActions).retry);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '0');
  await settle(page);
}

async function changeRival(page, expected) {
  const d = await droneState(page);
  if (page.viewportSize().width > page.viewportSize().height) {
    await page.touchscreen.tap(...JSON.parse(d.droneMenu));
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMenuOpen === '1');
    await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  }
  const point = JSON.parse((await droneState(page)).droneRivalButton);
  assert(point[0] > 0 && point[0] < page.viewportSize().width);
  assert(point[1] > 0 && point[1] < page.viewportSize().height);
  await page.touchscreen.tap(...point);
  await page.waitForFunction((rival) => {
    const d = document.querySelector('canvas')?.dataset;
    return d?.droneRival === rival && d.droneFrame === '0' && d.droneMenuOpen === '0';
  }, expected);
  await settle(page);
}

export async function checkBestGhost(page, base, shots) {
  assert.equal((await droneState(page)).droneRival, 'auto');
  await page.touchscreen.tap(...JSON.parse((await droneState(page)).droneActions).pause);
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 50);
  const first = await flyManualCourse(page);
  await retry(page);
  let d = await droneState(page);
  assert.equal(d.droneRival, 'best');
  assert.equal(d.droneGhostFinish, (Number(first.droneFrame) * 0.05).toFixed(2));
  assert.equal(d.droneBestSplit, '');
  if (shots) await page.screenshot({ path: `${shots}/drone-ghost-ready.png` });

  // Both the portrait button and the compact landscape menu really restart
  // against the selected rival, leaving the local best intact.
  await changeRival(page, 'auto');
  await changeRival(page, 'best');
  await page.setViewportSize({ width: 844, height: 390 });
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.droneMenu));
  await changeRival(page, 'auto');
  await changeRival(page, 'best');

  let moment, shared;
  const faster = await flyManualCourse(page, async (progress) => {
    if (progress.droneGates !== '2') return;
    await page.keyboard.press('Space');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
    await page.keyboard.press('3');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneCamera === 'course');
    moment = await droneState(page);
    assert.equal(moment.droneGhostFrame, moment.droneFrame, 'the best ghost ran ahead');
    assert.equal(moment.droneRival, 'best');
    assert(Number(moment.droneBestSplit.split(',')[1]) < 0, 'the deliberate hover should put the best behind');
    if (shots) await page.screenshot({ path: `${shots}/drone-ghost-racing.png` });
    await page.waitForTimeout(250);
    assert.equal((await droneState(page)).droneGhostPosition, moment.droneGhostPosition, 'paused ghost moved');

    const receiverContext = await page.context().browser().newContext({
      deviceScaleFactor: await page.evaluate(() => devicePixelRatio),
      viewport: page.viewportSize(),
      hasTouch: true,
    });
    const receiver = await receiverContext.newPage();
    await receiver.bringToFront();
    const errors = [];
    receiver.on('pageerror', (error) => errors.push(error.message));
    try {
      await receiver.goto(`${base}?${moment.droneShare}`, { waitUntil: 'load' });
      await waitDrone(receiver, () => {
        const d = document.querySelector('canvas')?.dataset;
        return d?.droneSeeking === '0' && Number(d.droneFrame) > 0;
      });
      shared = await droneState(receiver);
      for (const key of ['droneFrame', 'dronePosition', 'droneGhostFrame', 'droneGhostPosition', 'droneGhostGates', 'droneGhostFinish', 'droneBestSplit']) {
        assert.equal(shared[key], moment[key], `shared ${key} changed`);
      }
      assert.equal(shared.droneRival, 'best');
      assert.equal(shared.droneReplay, '1');
      assert.equal(shared.droneBestSteps, '0');
      assert.equal(await receiver.evaluate((key) => localStorage.getItem(key), storageKey), null);
      if (shots) await receiver.screenshot({ path: `${shots}/drone-ghost-shared.png` });
      await receiver.keyboard.press('p');
      await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.dronePhoto === '1');
      assert((await droneState(receiver)).droneShare.includes('&rival=best&best_inputs='));
      if (shots) await receiver.screenshot({ path: `${shots}/drone-ghost-photo.png` });
      await receiver.keyboard.press('Escape');
      await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
      await retry(receiver);
      assert.equal((await droneState(receiver)).droneRival, 'auto', 'received ghost became a local best');
      assert.deepEqual(errors, []);
    } finally { await receiverContext.close(); }
    await page.bringToFront();
    await page.keyboard.press('Space');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '1');
    return true;
  });
  assert(moment && shared, 'no checkpoint comparison was exercised');
  assert.equal(faster.droneNewBest, '1');
  assert(Number(faster.droneFrame) < Number(first.droneFrame));
  assert.equal(faster.droneGhostFinish, (Number(first.droneFrame) * 0.05).toFixed(2), 'a new best changed the current rival');
  assert(Number(faster.droneBestSplit.split(',')[1]) < 0);
  if (shots) await page.screenshot({ path: `${shots}/drone-ghost-finish.png` });
  await retry(page);
  assert.equal((await droneState(page)).droneGhostFinish, (Number(faster.droneFrame) * 0.05).toFixed(2));
  if (shots) await writeFile(`${shots}/drone-ghost-flight.json`, JSON.stringify({ first, moment, shared, faster }, null, 2) + '\n');
}

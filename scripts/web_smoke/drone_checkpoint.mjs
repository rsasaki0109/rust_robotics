// Observe feedback from actual gate crossings, including a fresh shared replay.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { droneState, flyManualCourse, waitDrone } from './drone_manual.mjs';
import { photoAction, savePhoto } from './drone_photo.mjs';

const clearKeys = ['droneClearGate', 'droneClearTime', 'droneClearAge', 'droneClearFinish'];
const clearState = (d) => Object.fromEntries(clearKeys.map((key) => [key, d[key]]));
const settled = (page) => page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));

function clockMatches(d) {
  assert.equal(d.droneClearGate, d.droneGates);
  assert(Number(d.droneClearTime) > 0);
  assert(Number(d.droneClearAge) >= 0 && Number(d.droneClearAge) < 1);
  assert(Math.abs(Number(d.droneClearTime) + Number(d.droneClearAge) - Number(d.droneFrame) * 0.05) < 0.001);
}

export async function checkGateFeedback(page, base, shots) {
  assert.equal((await droneState(page)).droneClearGate, '');
  await page.mouse.click(800, 350);
  let moment, shared, expired;
  const finish = await flyManualCourse(page, async (progress) => {
    if (progress.droneGates !== '1' || moment) return;
    await page.waitForFunction((frame) => Number(document.querySelector('canvas')?.dataset.droneFrame) >= frame + 4,
      Number(progress.droneFrame), { timeout: 15000 });
    await page.keyboard.press('Space');
    for (const key of ['w', 's', 'a', 'd', 'e', 'q']) await page.keyboard.up(key);
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
    await page.keyboard.press('3');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneCamera === 'course');
    await settled(page);
    moment = await droneState(page);
    clockMatches(moment);
    assert.equal(moment.droneClearFinish, '0');
    if (shots) await page.screenshot({ path: `${shots}/drone-checkpoint-clear.png` });
    await page.waitForTimeout(250);
    assert.deepEqual(clearState(await droneState(page)), clearState(moment), 'paused feedback used wall time');
    await page.keyboard.press('p');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '1');
    await savePhoto(page, 16 / 9, shots && `${shots}/drone-checkpoint-photo.png`);
    await photoAction(page, 'close');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
    assert.deepEqual(clearState(await droneState(page)), clearState(moment), 'photo mode advanced the feedback');

    const receiverContext = await page.context().browser().newContext({
      deviceScaleFactor: await page.evaluate(() => devicePixelRatio),
      viewport: page.viewportSize(),
    });
    const receiver = await receiverContext.newPage();
    await receiver.bringToFront();
    try {
      await receiver.goto(`${base}?${moment.droneShare}`, { waitUntil: 'load' });
      await waitDrone(receiver, () => {
        const d = document.querySelector('canvas')?.dataset;
        return d?.droneSeeking === '0' && Number(d.droneFrame) > 0;
      });
      shared = await droneState(receiver);
      assert.deepEqual(clearState(shared), clearState(moment), 'shared gate feedback changed');
      assert.equal(shared.dronePosition, moment.dronePosition);
      assert.equal(shared.droneReplay, '1');
      assert.equal(shared.droneBestSteps, '0');
      if (shots) await receiver.screenshot({ path: `${shots}/drone-checkpoint-shared.png` });
      const earlier = new URLSearchParams(moment.droneShare);
      earlier.set('frame', '0'); earlier.set('playing', '0');
      await receiver.goto(`${base}?${earlier}`, { waitUntil: 'load' });
      await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneSeeking === '0');
      assert.equal((await droneState(receiver)).droneClearGate, '', 'a future crossing leaked into frame zero');
    } finally { await receiverContext.close(); }
    await page.bringToFront();

    // Brake with a real reverse command so the same gate remains the last
    // crossed one while its one-second simulation-time feedback expires.
    await page.keyboard.down('s');
    await page.keyboard.press('Space');
    await page.waitForFunction((frame) => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.droneClearGate === '' && Number(d.droneFrame) >= frame + 21;
    }, Number(moment.droneFrame), { timeout: 15000 });
    await page.keyboard.press('Space');
    await page.keyboard.up('s');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
    expired = await droneState(page);
    assert.equal(expired.droneGates, '1');
    assert.equal(expired.droneClearGate, '');
    await page.keyboard.press('r');
    await waitDrone(page, () => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.droneFrame === '0' && d.dronePlaying === '0' && d.droneClearGate === '';
    });
    return true;
  });
  assert(moment && shared && expired);
  clockMatches(finish);
  assert.equal(finish.droneClearGate, '5');
  assert.equal(finish.droneClearFinish, '1');
  if (shots) {
    await page.screenshot({ path: `${shots}/drone-checkpoint-finish.png` });
    await writeFile(`${shots}/drone-checkpoint-flight.json`, JSON.stringify({ moment, shared, expired, finish }, null, 2) + '\n');
  }
}

export async function checkGateFeedbackPhone(page, base, shots) {
  assert.equal((await droneState(page)).droneClearGate, '');
  await page.locator('canvas').click({ position: { x: 195, y: 350 } });
  await page.keyboard.press('3');
  await page.keyboard.press('Space');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneClearGate === '1');
  const crossed = Number((await droneState(page)).droneFrame);
  await page.waitForFunction((frame) => Number(document.querySelector('canvas')?.dataset.droneFrame) >= frame + 4,
    crossed, { timeout: 15000 });
  await page.keyboard.press('Space');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  await settled(page);
  const moment = await droneState(page);
  clockMatches(moment);
  assert.equal(moment.droneMode, 'auto');
  if (shots) await page.screenshot({ path: `${shots}/drone-checkpoint-portrait.png` });
  await page.setViewportSize({ width: 844, height: 390 });
  await settled(page);
  assert.deepEqual(clearState(await droneState(page)), clearState(moment), 'rotation advanced the feedback');
  if (shots) await page.screenshot({ path: `${shots}/drone-checkpoint-landscape.png` });
  await page.waitForTimeout(250);
  assert.deepEqual(clearState(await droneState(page)), clearState(moment));
  const earlier = new URLSearchParams(moment.droneShare);
  earlier.set('frame', '0'); earlier.set('playing', '0');
  await page.goto(`${base}?${earlier}`, { waitUntil: 'load' });
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneSeeking === '0');
  assert.equal((await droneState(page)).droneClearGate, '');
}

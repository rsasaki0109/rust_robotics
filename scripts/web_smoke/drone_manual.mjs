// Functional checks and a keyboard driver shared by smoke tests and capture.
// The driver sends ordinary held keys; it never writes a simulation state.
import assert from 'node:assert/strict';

const canvas = '#rust_robotics_playground';
export const droneState = (page) => page.locator(canvas).evaluate((c) => ({ ...c.dataset }));

export async function waitDrone(page, predicate, timeout = 30000) {
  await page.waitForFunction(predicate, null, { timeout });
}

export async function flyManualCourse(page, onProgress = () => {}) {
  const centers = [[4, 0, 2.8], [8, 0, 4], [12, 0.8, 5.2], [16, -0.6, 5.6], [20, 0, 4]];
  const held = new Set();
  let lastGate = -1;
  const started = Date.now();
  try {
    while (Date.now() - started < 90000) {
      const d = await droneState(page);
      if (d.droneStatus !== 'flying') {
        assert.equal(d.droneStatus, 'finished', `manual flight ended: ${d.droneStatus}`);
        assert.equal(d.droneGates, '5');
        return d;
      }
      assert.equal(d.droneCourse, 'climb');
      assert.equal(d.droneReplay, '0');
      const gate = Number(d.droneGates);
      if (gate !== lastGate) { await onProgress(d); lastGate = gate; }
      const position = d.dronePosition.split(',').map(Number);
      const velocity = d.droneVelocity.split(',').map(Number);
      const target = centers[gate];
      const error = target.map((value, i) => value - position[i] - velocity[i] * 0.4);
      const forward = target[0] - position[0] < -0.5 ? -1
        : target[0] - position[0] < 1.5 && (Math.abs(error[1]) > 0.6 || Math.abs(error[2]) > 0.6) ? 0 : 1;
      const axis = (value) => value > 0.25 ? 1 : value < -0.25 ? -1 : 0;
      const desired = new Set([
        forward > 0 ? 'w' : forward < 0 ? 's' : null,
        axis(error[1]) > 0 ? 'a' : axis(error[1]) < 0 ? 'd' : null,
        axis(error[2]) > 0 ? 'e' : axis(error[2]) < 0 ? 'q' : null,
      ].filter(Boolean));
      for (const key of held) if (!desired.has(key)) { await page.keyboard.up(key); held.delete(key); }
      for (const key of desired) if (!held.has(key)) { await page.keyboard.down(key); held.add(key); }
      // Observe progress, rather than assuming a browser renders in real time.
      await page.waitForFunction((frame) => {
        const d = document.getElementById('rust_robotics_playground')?.dataset;
        return Number(d?.droneFrame) >= frame + 2 || d?.droneStatus !== 'flying';
      }, Number(d.droneFrame), { timeout: 15000 });
    }
    throw new Error('manual keyboard flight did not finish within 90 seconds');
  } finally {
    for (const key of held) await page.keyboard.up(key);
  }
}

export async function checkManualDesktop(page, base, shots) {
  let d = await droneState(page);
  assert.equal(d.droneFrame, '0');
  assert.equal(d.dronePlaying, '0', 'the race must wait for its human pilot');
  await page.mouse.click(800, 350);
  await page.keyboard.down('w');
  await page.keyboard.down('e');
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 8);
  await page.keyboard.up('e');
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneGates) >= 1);
  await page.keyboard.up('w');
  await page.keyboard.press('Space');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  d = await droneState(page);
  assert.equal(d.droneInput, '0,0,0');
  assert.equal(d.droneGhostFrame, d.droneFrame, 'opponent must use the same simulation clock');
  const position = d.dronePosition.split(',').map(Number);
  assert(position[0] > 3 && position[2] > 2.3, 'held keys must physically move and lift the aircraft');
  await page.waitForTimeout(250);
  assert.equal((await droneState(page)).dronePosition, d.dronePosition, 'paused position changed');

  const replay = await page.context().newPage();
  try {
    await replay.goto(`${base}?${d.droneShare}`, { waitUntil: 'load' });
    await waitDrone(replay, () => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.droneSeeking === '0' && Number(d.droneFrame) > 0;
    });
    const restored = await droneState(replay);
    assert.equal(restored.droneFrame, d.droneFrame);
    assert.equal(restored.dronePosition, d.dronePosition);
    assert.equal(restored.droneGhostGates, d.droneGhostGates);
    assert.equal(restored.droneReplay, '1');
    await replay.keyboard.down('w');
    await replay.waitForTimeout(250);
    await replay.keyboard.up('w');
    assert.equal((await droneState(replay)).dronePosition, d.dronePosition, 'a new input changed a shared replay');
    await replay.keyboard.press('Space');
    await waitDrone(replay, () => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.dronePlaying === '1' && Number(d.droneFrame) < 10;
    });
    await waitDrone(replay, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
    assert.equal((await droneState(replay)).dronePosition, d.dronePosition, 'replay appended unrecorded commands');
  } finally { await replay.close(); }
  await page.bringToFront();
  await page.mouse.click(800, 350);
  await page.keyboard.press('Space');
  await flyManualCourse(page);
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.droneGhostFinish));
  d = await droneState(page);
  assert.equal(d.droneGates, '5');
  assert(Number(d.droneGhostFinish) > 0);
  if (shots) await page.screenshot({ path: `${shots}/drone-manual-finish.png` });
  await page.keyboard.press('r');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '0');
  d = await droneState(page);
  assert.equal(d.droneGenerated, '0');
  assert.equal(d.droneGates, '0');
  assert.equal(d.droneReplay, '0');
  assert.equal(d.dronePlaying, '0');
}

export async function checkManualPhone(page) {
  const d = await droneState(page);
  const pad = JSON.parse(d.dronePad);
  const actions = JSON.parse(d.droneActions);
  const cdp = await page.context().newCDPSession(page);
  const point = (id, pos) => ({ id, x: pos[0], y: pos[1], radiusX: 8, radiusY: 8, force: 1 });
  const forward = point(1, pad.forward), up = point(2, pad.up);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [forward, up] });
  await waitDrone(page, () => {
    const d = document.querySelector('canvas')?.dataset;
    return d?.droneInput === '1,0,1' && Number(d.droneFrame) >= 8;
  });
  // CDP touchEnd lists the changed (released) contact.
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [up] });
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneInput === '1,0,0');
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneInput === '0,0,0');
  let flying = await droneState(page);
  assert(Number(flying.dronePosition.split(',')[0]) > 0.2);
  assert(Number(flying.dronePosition.split(',')[2]) > 2.2);
  await page.touchscreen.tap(...actions.pause);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  flying = await droneState(page);
  await page.waitForTimeout(250);
  assert.equal((await droneState(page)).dronePosition, flying.dronePosition);
  await page.touchscreen.tap(...actions.retry);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '0');
  assert.equal((await droneState(page)).dronePlaying, '0');
}

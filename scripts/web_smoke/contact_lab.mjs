// Manual contact experiments through actual canvas mouse/touch input.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => Object.fromEntries(
  Object.entries(c.dataset).filter(([key]) => key.startsWith('lab'))));
const settle = (page) => page.waitForTimeout(100);
const receipts = new WeakMap();
const close = (actual, expected, message) => assert(Math.abs(actual - expected) < 8e-8, `${message}: ${actual} != ${expected}`);

function point(d, name, fraction) {
  if (fraction !== 'contact') return JSON.parse(d.labActions)[name];
  const [x, y, width, height] = JSON.parse(d.labRect);
  const [px, py] = JSON.parse(d.labContact), theta = JSON.parse(d.labPose)[2];
  const scale = width / 0.32;
  return [x + width / 2 + (Math.cos(theta) * px - Math.sin(theta) * py) * scale,
    y + height / 2 - (Math.sin(theta) * px + Math.cos(theta) * py) * scale];
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
      const pointer = JSON.parse(d?.labPointer ?? 'null');
      return Number(d?.labUiRevision) > revision && pointer && Math.hypot(pointer[0] - position[0], pointer[1] - position[1]) < 0.75;
    }, { revision: Number(before.labUiRevision), position });
    const after = await state(page);
    const current = point(after, name, fraction);
    if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) return false;
  }
  return JSON.parse((await state(page)).labHits)[name] === true;
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
    if (!d || Number(d.labUiRevision) <= revision) return false;
    const y = JSON.parse(d.labActions)[name][1];
    return delta < 0 ? y > previous + 1 : y < previous - 1;
  }, { revision: Number(before.labUiRevision), name, fraction, previous, delta });
}

async function action(page, name, touch, fraction) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const target = point(await state(page), name, fraction);
    assert(target, `missing lab action: ${name}`);
    const { width, height } = page.viewportSize();
    if (target[0] >= 0 && target[0] <= width && target[1] >= 64 && target[1] <= height - 24) {
      await settle(page);
      if (!await aim(page, name, target, fraction)) continue;
      const before = await state(page);
      const current = point(before, name, fraction);
      if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) continue;
      if (!JSON.parse(before.labHits)[name]) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      await page.waitForFunction((count) => Number(document.querySelector('canvas')?.dataset.labActionCount) > count, Number(before.labActionCount));
      const after = await state(page);
      assert.equal(after.labLastAction, name, `input hit a different control than ${name}`);
      assert.equal(Number(after.labActionCount), Number(before.labActionCount) + 1, `${name} was handled more than once`);
      receipts.get(page)?.push({ name, count: Number(after.labActionCount), regime: after.labRegime,
        pose: JSON.parse(after.labPose), position: current, hit: JSON.parse(before.labHits)[name] });
      return after;
    }
    await page.mouse.move(Math.min(width - 20, Math.max(20, target[0])), height - 60);
    await scrollAction(page, name, fraction);
    await settle(page);
  }
  throw Error(`lab action is unreachable: ${name}`);
}

function checkPhysics(d) {
  const [vx, vy, omega] = JSON.parse(d.labTwist);
  const [px, py] = JSON.parse(d.labContact);
  const realized = JSON.parse(d.labRealized);
  close(realized[0], vx - omega * py, 'rigid contact x velocity');
  close(realized[1], vy + omega * px, 'rigid contact y velocity');
  const pose = JSON.parse(d.labPose), next = JSON.parse(d.labNext);
  close(next[0], pose[0] + 0.1 * (Math.cos(pose[2]) * vx - Math.sin(pose[2]) * vy), 'next x');
  close(next[1], pose[1] + 0.1 * (Math.sin(pose[2]) * vx + Math.cos(pose[2]) * vy), 'next y');
  close(next[2], pose[2] + 0.1 * omega, 'next angle');
  const n = JSON.parse(d.labNormal);
  const request = JSON.parse(d.labRequested);
  close(realized[0] * n[0] + realized[1] * n[1], request[0] * n[0] + request[1] * n[1], 'normal speed continuity');
  if (d.labRegime === 'Sticking') realized.forEach((v, i) => close(v, request[i], 'no relative contact motion while sticking'));
}

async function sceneVisible(page) {
  for (let i = 0; i < 14; i++) {
    const d = await state(page), r = JSON.parse(d.labRect);
    if (r[1] >= 48 && r[1] + r[3] <= page.viewportSize().height - 12) {
      if (await aim(page, 'aim', point(d, 'aim', 'contact'), 'contact')) return JSON.parse((await state(page)).labRect);
      continue;
    }
    await page.mouse.move(page.viewportSize().width / 2, 120);
    await page.mouse.wheel(0, -300);
    await page.waitForFunction(({ revision, y }) => {
      const d = document.querySelector('canvas')?.dataset;
      return Number(d?.labUiRevision) > revision && JSON.parse(d.labRect)[1] > y + 1;
    }, { revision: Number(d.labUiRevision), y: r[1] });
  }
  throw Error('contact scene is unreachable');
}

async function dragVelocity(page, touch) {
  const r = await sceneVisible(page), before = await state(page);
  const [px, py] = JSON.parse(before.labContact);
  const scale = r[2] / 0.32;
  const center = [r[0] + r[2] / 2, r[1] + r[3] / 2];
  // The experiment was reset, so body axes agree with the world axes.
  const start = [center[0] + px * scale, center[1] - py * scale].map(Math.floor);
  const end = [start[0] + 0.025 * scale, start[1] - 0.04 * scale].map(Math.floor);
  if (touch) {
    const cdp = await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ x: start[0], y: start[1] }] });
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ x: end[0], y: end[1] }] });
    await page.waitForFunction(() => JSON.parse(document.querySelector('canvas').dataset.labCommand)[3] > 0.02);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await cdp.detach();
  } else {
    await page.mouse.move(...start); await page.mouse.down();
    await page.mouse.move(...end, { steps: 4 });
    await page.waitForFunction(() => JSON.parse(document.querySelector('canvas').dataset.labCommand)[3] > 0.02);
    await page.mouse.up();
  }
  await page.waitForFunction((count) => Number(document.querySelector('canvas').dataset.labActionCount) > count, Number(before.labActionCount));
  const after = await state(page), cmd = JSON.parse(after.labCommand);
  assert.equal(after.labLastAction, 'aim');
  assert.equal(Number(after.labActionCount), Number(before.labActionCount) + 1);
  assert(Math.abs(cmd[2] - 0.025) < 0.003 && Math.abs(cmd[3] - 0.04) < 0.003, 'velocity drag has wrong scale or axes');
  checkPhysics(after);
  receipts.get(page).push({ name: 'aim', count: Number(after.labActionCount), command: cmd, touch });
  return after;
}

export async function checkContactLab(page, base, shots, touch = false, slow = false) {
  receipts.set(page, []);
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.labMode === 'contact');
  const initial = await state(page);
  assert.equal(initial.labRegime, 'Sticking'); checkPhysics(initial);
  const center = await action(page, 'push', touch);
  assert.deepEqual(JSON.parse(center.labPose), JSON.parse(initial.labNext), 'push must execute the displayed preview');
  assert.equal(center.labSteps, '1');
  await page.waitForTimeout(450);
  assert.equal((await state(page)).labPose, center.labPose, 'manual experiment moved without a push');
  let off = await action(page, 'edge', touch);
  assert(Math.abs(JSON.parse(off.labTwist)[2]) > 0.1, 'off-center push must turn');
  off = await action(page, 'push', touch); checkPhysics(off);
  let glancing = await action(page, 'glance', touch);
  const low = await action(page, 'low', touch);
  assert.equal(low.labRegime, 'Sliding +t'); checkPhysics(low);
  assert(JSON.parse(low.labRequested)[1] - JSON.parse(low.labRealized)[1] > 0.02);
  const high = await action(page, 'high', touch);
  assert.equal(high.labRegime, 'Sticking'); checkPhysics(high);
  const front = await action(page, 'front', touch); checkPhysics(front);
  assert.equal(JSON.parse(front.labCommand)[0], 2);
  assert(JSON.parse(front.labTwist)[0] < 0, 'front contact must push toward -x');
  const release = await action(page, 'release', touch);
  assert.equal(release.labRegime, 'Separated');
  const released = await action(page, 'push', touch);
  assert.equal(released.labPose, release.labPose, 'released pusher moved the box');
  await action(page, 'center', touch);
  const aimed = await dragVelocity(page, touch);
  const pushed = await action(page, 'push', touch);
  assert.deepEqual(JSON.parse(pushed.labPose), JSON.parse(aimed.labNext));
  const saved = await state(page);
  // Mode switches preserve the manual experiment while the original MPPI UI works.
  const delivery = await action(page, 'delivery', touch); assert.equal(delivery.labMode, 'delivery');
  const returned = await action(page, 'contact', touch);
  assert.equal(returned.labPose, saved.labPose); assert.equal(returned.labCommand, saved.labCommand);
  const reset = await action(page, 'reset-pose', touch);
  assert.deepEqual(JSON.parse(reset.labPose), [0, 0, 0]);
  assert.equal(reset.labCommand, saved.labCommand);
  glancing = await action(page, 'glance', touch); await action(page, 'low', touch);
  await sceneVisible(page);
  if (shots) await page.screenshot({ path: join(shots, `contact-lab${touch ? '-phone' : ''}${slow ? '-slow' : ''}-scene.png`) });
  const inputs = receipts.get(page);
  let paintTimes;
  if (slow) {
    // Capture the interaction document's paints before sharing reloads it.
    paintTimes = await page.evaluate(() => window.__slowPaintTimes);
    assert(paintTimes.length >= 3 && paintTimes.slice(1).every((v, i) => v - paintTimes[i] >= 350), 'slow test did not cap paints');
  }
  await page.goto(`${base}?${saved.labShare}`, { waitUntil: 'load' });
  await page.waitForFunction(() => document.querySelector('canvas')?.dataset.labMode === 'contact');
  const shared = await state(page);
  for (const key of ['labPose', 'labCommand', 'labMu', 'labTwist', 'labNext', 'labRegime']) assert.equal(shared[key], saved[key], `${key} changed in the shared experiment`);
  assert.equal(shared.labSteps, '0');
  checkPhysics(shared);
  if (shots) await writeFile(join(shots, `contact-lab${touch ? '-phone' : ''}${slow ? '-slow' : ''}.json`), JSON.stringify({ initial, low, high, saved, shared, inputs, paintTimes }, null, 2));
}


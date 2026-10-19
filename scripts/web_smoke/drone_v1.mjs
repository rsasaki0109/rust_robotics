// Start at the published site's project path and use real buttons/keys/touches.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { droneState, flyManualCourse, waitDrone } from './drone_manual.mjs';
import { photoAction, savePhoto } from './drone_photo.mjs';

export async function openFromShowcase(page, home, shots, name) {
  // The site must remain usable when an optional external font is unavailable.
  await page.route('https://fonts.googleapis.com/**', (route) => route.fulfill({ contentType: 'text/css', body: '' }));
  await page.goto(home, { waitUntil: 'load' });
  await page.locator('#play-drone').waitFor();
  assert(await page.locator('.hero-shot img').evaluate((img) => img.complete && img.naturalWidth > 0));
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'showcase overflows horizontally');
  const href = new URL(await page.locator('#play-drone').getAttribute('href'), home);
  assert.equal(href.pathname, '/rust_robotics/playground/');
  assert.equal(href.searchParams.get('intro'), '1');
  assert.equal(href.searchParams.get('mode'), 'manual');
  if (shots) await page.screenshot({ path: `${shots}/${name}-home.png` });
  await page.locator('#play-drone').click();
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneGuide === '1');
}

export async function guideAction(page, name, touch = false) {
  const d = await droneState(page);
  const point = JSON.parse(d.droneGuideActions)[name];
  if (touch) await page.touchscreen.tap(...point);
  else await page.mouse.click(...point);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneGuide === '0');
  await page.waitForTimeout(100);
}

function guideFits(d, viewport) {
  const [x, y, w, h] = JSON.parse(d.droneGuideRect);
  assert(x >= 0 && y >= 0 && x + w <= viewport.width + 1 && y + h <= viewport.height + 1, 'guide extends beyond the screen');
  for (const [name, [bx, by]] of Object.entries(JSON.parse(d.droneGuideActions))) {
    assert(bx >= x && bx <= x + w && by - 20 >= y && by + 20 <= y + h, `${name} button falls outside the guide`);
  }
}

export async function checkFlightV1(page, base, shots) {
  let d = await droneState(page);
  guideFits(d, page.viewportSize());
  assert.equal(d.droneFrame, '0');
  assert.equal(d.droneGenerated, '0');
  assert.equal(d.dronePlaying, '0');
  await page.keyboard.press('Space'); await page.keyboard.press('r');
  await page.keyboard.down('w'); await page.waitForTimeout(200); await page.keyboard.up('w');
  assert.equal((await droneState(page)).droneGenerated, '0', 'the guide advanced physics');
  if (shots) await page.screenshot({ path: `${shots}/drone-v1-guide.png` });
  await guideAction(page, 'fly');
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.dronePad));
  assert.equal((await droneState(page)).dronePlaying, '0', 'Ready started the clock before input');
  let captured = false;
  const finish = await flyManualCourse(page, async (state) => {
    if (shots && Number(state.droneGates) >= 2 && !captured) {
      await page.screenshot({ path: `${shots}/drone-v1-flight.png` }); captured = true;
    }
  });
  assert.equal(finish.droneGates, '5');
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.droneGhostFinish));
  const finished = await droneState(page);
  if (shots) await page.screenshot({ path: `${shots}/drone-v1-finish.png` });
  await page.keyboard.press('p');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '1');
  await savePhoto(page, 16 / 9, shots && `${shots}/drone-v1-export.png`);
  await photoAction(page, 'close');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
  d = await droneState(page);
  assert.equal(d.dronePosition, finished.dronePosition);
  assert(!new URLSearchParams(d.droneShare).has('intro'), 'a shared replay reopens the intro');
  await page.context().grantPermissions(['clipboard-read', 'clipboard-write']);
  await page.mouse.click(page.viewportSize().width - 32, 22);
  await page.waitForFunction(async () => {
    const text = await navigator.clipboard.readText();
    return text.includes('inputs=') && text.includes('tab=drone');
  });
  const copied = await page.evaluate(() => navigator.clipboard.readText());
  assert.equal(new URL(copied).pathname, '/rust_robotics/playground/');
  assert(!new URL(copied).searchParams.has('intro'));
  const receiver = await page.context().newPage();
  let replay;
  try {
    await receiver.goto(copied, { waitUntil: 'load' });
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished' && document.querySelector('canvas')?.dataset.droneSeeking === '0');
    const restored = await droneState(receiver);
    replay = restored;
    assert.equal(restored.droneGuide, '0');
    assert.equal(restored.droneReplay, '1');
    assert.equal(restored.dronePosition, d.dronePosition);
    assert.equal(restored.droneGates, '5');
    assert.equal(restored.droneFrame, d.droneFrame);
  } finally { await receiver.close(); }
  await page.bringToFront();
  if (shots) await writeFile(`${shots}/drone-v1-flight.json`, JSON.stringify({ finish: finished, replay, query: d.droneShare }, null, 2) + '\n');
  return d;
}

async function holdFlight(page) {
  const before = await droneState(page);
  const pad = JSON.parse(before.dronePad);
  const cdp = await page.context().newCDPSession(page);
  const point = (id, pos) => ({ id, x: pos[0], y: pos[1], radiusX: 7, radiusY: 7, force: 1 });
  try {
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [point(1, pad.forward), point(2, pad.up)] });
    await page.waitForFunction((frame) => Number(document.querySelector('canvas')?.dataset.droneFrame) >= frame + 8, Number(before.droneFrame), { timeout: 15000 });
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  } finally { await cdp.detach(); }
}

function controlsFit(d, viewport) {
  const pads = Object.values(JSON.parse(d.dronePad));
  const actions = Object.values(JSON.parse(d.droneActions));
  for (const [x, y] of [...pads, ...actions]) {
    assert(x >= 22 && y >= 20 && x <= viewport.width - 22 && y <= viewport.height - 20, 'a flight control is outside the viewport');
  }
  for (const [x, y] of pads) for (const [bx, by] of actions) {
    assert(Math.abs(x - bx) >= 62 || Math.abs(y - by) >= 42, 'a flight button overlaps playback');
  }
}

async function checkOptions(page) {
  if (page.viewportSize().width < 500) return;
  let d = await droneState(page);
  await page.touchscreen.tap(...JSON.parse(d.droneMenu));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMenuOpen === '1');
  await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  d = await droneState(page);
  await page.touchscreen.tap(...JSON.parse(d.droneMenuCourse));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneCamera === 'course' && document.querySelector('canvas')?.dataset.droneMenuOpen === '0');
  d = await droneState(page);
  await page.touchscreen.tap(...JSON.parse(d.droneMenu));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMenuOpen === '1');
  await page.evaluate(() => new Promise((resolve) => requestAnimationFrame(() => requestAnimationFrame(resolve))));
  const frozen = await droneState(page);
  await page.keyboard.press('Escape');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMenuOpen === '0');
  d = await droneState(page);
  assert.equal(d.droneCinema, '1', 'Escape closed the flight view along with the menu');
  assert.equal(d.droneFrame, frozen.droneFrame);
  assert.equal(d.droneInput, '0,0,0', 'a menu click reached a flight button');
}

export async function checkPhoneV1(page, shots, name) {
  let d = await droneState(page);
  guideFits(d, page.viewportSize());
  if (shots) await page.screenshot({ path: `${shots}/${name}-guide.png` });
  await guideAction(page, 'fly', true);
  await waitDrone(page, () => Boolean(document.querySelector('canvas')?.dataset.dronePad));
  d = await droneState(page);
  controlsFit(d, page.viewportSize());
  await checkOptions(page);
  await holdFlight(page);
  d = await droneState(page);
  assert(Number(d.dronePosition.split(',')[0]) > 0.25 && Number(d.dronePosition.split(',')[2]) > 2.3, 'touches did not move and lift the aircraft');
  await page.touchscreen.tap(...JSON.parse(d.droneGuideOpen));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneGuide === '1');
  const frozen = await droneState(page);
  assert.equal(frozen.dronePlaying, '0');
  assert.equal(frozen.droneInput, '0,0,0');
  await page.waitForTimeout(200);
  assert.equal((await droneState(page)).dronePosition, frozen.dronePosition);
  await guideAction(page, 'fly', true);
  d = await droneState(page);
  assert.equal(d.dronePosition, frozen.dronePosition);
  assert.equal(d.droneReplay, '0');
  assert.equal(d.dronePlaying, '0');
  await page.touchscreen.tap(...JSON.parse(d.droneActions).pause);
  await holdFlight(page);
  d = await droneState(page);
  await page.touchscreen.tap(...JSON.parse(d.droneActions).pause);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  assert(Number((await droneState(page)).droneFrame) > Number(frozen.droneFrame));
  await page.setViewportSize(page.viewportSize().width < 500 ? { width: 667, height: 375 } : { width: 390, height: 844 });
  await page.waitForTimeout(400);
  d = await droneState(page);
  controlsFit(d, page.viewportSize());
  assert.equal(d.dronePlaying, '0');
  assert.equal(d.droneInput, '0,0,0');
  await checkOptions(page);
  if (shots) await page.screenshot({ path: `${shots}/${name}-rotated.png` });
  await page.touchscreen.tap(...JSON.parse(d.droneGuideOpen));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneGuide === '1');
  await guideAction(page, 'watch', true);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMode === 'auto');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished', 45000);
  assert.equal((await droneState(page)).droneGates, '5');
}

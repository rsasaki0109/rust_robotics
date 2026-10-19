// Real camera gestures and PNG downloads; no simulation state is injected.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { droneState, waitDrone } from './drone_manual.mjs';

export async function photoAction(page, name, touch = false) {
  const d = await droneState(page);
  const point = JSON.parse(d.dronePhotoActions)[name];
  assert(point, `missing photo action ${name}`);
  if (touch) await page.touchscreen.tap(...point);
  else await page.mouse.click(...point);
  await page.waitForTimeout(100);
}

export async function orbitPhoto(page, touch = false) {
  const before = await droneState(page);
  const [x, y, w, h] = JSON.parse(before.dronePhotoRect);
  const start = [x + w * 0.4, y + h * 0.4];
  const end = [start[0] + Math.min(80, w * 0.25), start[1] + 25];
  if (touch) {
    const cdp = await page.context().newCDPSession(page);
    const point = (pos) => ({ id: 1, x: pos[0], y: pos[1], radiusX: 7, radiusY: 7, force: 1 });
    try {
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [point(start)] });
      for (let i = 1; i <= 8; i++) {
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [point(start.map((v, axis) => v + (end[axis] - v) * i / 8))] });
        await page.waitForTimeout(30);
      }
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    } finally { await cdp.detach(); }
  } else {
    await page.mouse.move(...start); await page.mouse.down();
    for (let i = 1; i <= 8; i++) {
      await page.mouse.move(...start.map((v, axis) => v + (end[axis] - v) * i / 8));
      await page.waitForTimeout(30);
    }
    await page.mouse.up();
  }
  await page.waitForTimeout(150);
  const after = await droneState(page);
  assert.notEqual(after.dronePhotoOrbit, before.dronePhotoOrbit, 'orbit gesture did not move the camera');
  assert.equal(after.dronePosition, before.dronePosition, 'orbit moved the physical drone');
  assert.equal(after.droneFrame, before.droneFrame);
}

export async function savePhoto(page, ratio, path, touch = false) {
  const before = await droneState(page);
  const pending = page.waitForEvent('download', { timeout: 15000 });
  await photoAction(page, 'save', touch);
  const download = await pending;
  assert.equal(await download.failure(), null);
  assert.match(download.suggestedFilename(), /^rust-flight-(neon|climb|custom)-\d+-(wide|square|portrait)\.png$/);
  if (path) await download.saveAs(path);
  const bytes = await readFile(await download.path());
  assert.equal(bytes.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
  const width = bytes.readUInt32BE(16), height = bytes.readUInt32BE(20);
  assert(Math.abs(width / height - ratio) < 2 / height, `PNG has wrong aspect: ${width} × ${height}`);
  const rect = JSON.parse(before.dronePhotoRect);
  const scale = await page.evaluate(() => window.devicePixelRatio);
  assert(Math.abs(width - rect[2] * scale) < 2 && Math.abs(height - rect[3] * scale) < 2, 'PNG does not match the rendered crop at device resolution');
  const distinct = await page.evaluate(async (url) => {
    const image = new Image(); image.src = url; await image.decode();
    const canvas = document.createElement('canvas'); canvas.width = image.width; canvas.height = image.height;
    const ctx = canvas.getContext('2d'); ctx.drawImage(image, 0, 0);
    const data = ctx.getImageData(0, 0, canvas.width, canvas.height).data;
    const colors = new Set();
    for (let i = 0; i < data.length; i += 128) colors.add(`${data[i]},${data[i + 1]},${data[i + 2]}`);
    return colors.size;
  }, `data:image/png;base64,${bytes.toString('base64')}`);
  assert(distinct > 30, `PNG is empty or flat (${distinct} colors)`);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhotoSaving === '0');
  return { filename: download.suggestedFilename(), width, height, bytes: bytes.length, colors: distinct };
}

export async function checkPhotoDesktop(page, base, shots) {
  const original = await droneState(page);
  await page.mouse.click(...JSON.parse(original.dronePhotoOpen));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '1' && Boolean(document.querySelector('canvas')?.dataset.dronePhotoRect));
  await orbitPhoto(page);
  const orbit = await droneState(page);
  const area = JSON.parse(orbit.dronePhotoRect);
  await page.mouse.move(area[0] + area[2] * 0.5, area[1] + area[3] * 0.5);
  await page.mouse.wheel(0, -150);
  await page.waitForTimeout(350);
  const zoomed = (await droneState(page)).dronePhotoOrbit.split(',').map(Number);
  assert(zoomed[2] < 5, `scroll up did not zoom in (distance ${zoomed[2]})`);
  await photoAction(page, 'in');
  let d = await droneState(page);
  assert(Number(d.dronePhotoOrbit.split(',')[2]) < Number(orbit.dronePhotoOrbit.split(',')[2]));
  await photoAction(page, 'out');
  await photoAction(page, 'square');
  await savePhoto(page, 1, shots && `${shots}/drone-photo-square.png`);
  await photoAction(page, 'portrait');
  await savePhoto(page, 0.8, shots && `${shots}/drone-photo-portrait.png`);
  await photoAction(page, 'wide');
  await savePhoto(page, 16 / 9, shots && `${shots}/drone-photo-wide.png`);
  d = await droneState(page);
  const timeline = JSON.parse(d.dronePhotoActions).timeline;
  await page.mouse.click(timeline[0] - 40, timeline[1]);
  await page.waitForTimeout(150);
  d = await droneState(page);
  assert(Number(d.droneFrame) < Number(original.droneFrame), 'Moment slider did not choose an earlier physical state');
  assert.notEqual(d.dronePosition, original.dronePosition);
  assert.equal(d.droneGenerated, original.droneGenerated);
  await page.keyboard.press('Space'); await page.keyboard.press('r');
  await page.keyboard.down('w'); await page.waitForTimeout(200); await page.keyboard.up('w');
  assert.equal((await droneState(page)).droneFrame, d.droneFrame, 'flight keys advanced photo history');
  const receiver = await page.context().newPage();
  try {
    await receiver.goto(`${base}?${d.droneShare}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.dronePhoto === '1' && d.droneSeeking === '0' && Boolean(d.dronePhotoRect);
    });
    const restored = await droneState(receiver);
    for (const key of ['droneFrame', 'dronePosition', 'droneLayout', 'dronePhotoOrbit', 'dronePhotoFormat']) {
      assert.equal(restored[key], d[key], `photo link lost ${key}`);
    }
    assert.equal(restored.dronePlaying, '0');
    await receiver.keyboard.press('Escape');
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
  } finally { await receiver.close(); }
  await page.bringToFront();
  await photoAction(page, 'close');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
  const closed = await droneState(page);
  for (const key of ['droneFrame', 'dronePosition', 'droneGenerated']) assert.equal(closed[key], original[key], `photo mode changed original ${key}`);
  assert.equal(closed.dronePlaying, '0');
}

export async function checkPhotoPhone(page, base, shots) {
  await page.touchscreen.tap(220, 380);
  await page.keyboard.down('w'); await page.keyboard.down('e');
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 10);
  await page.keyboard.up('w'); await page.keyboard.up('e');
  await page.keyboard.press('p');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '1');
  const frozen = await droneState(page);
  assert.equal(frozen.droneInput, '0,0,0');
  await photoAction(page, 'portrait', true);
  const ready = await droneState(page);
  const rect = JSON.parse(ready.dronePhotoRect);
  const viewport = page.viewportSize();
  assert(rect[0] >= 0 && rect[1] >= 0 && rect[0] + rect[2] <= viewport.width && rect[1] + rect[3] <= viewport.height, 'phone composition extends outside the screen');
  await orbitPhoto(page, true);
  await photoAction(page, 'in', true);
  await savePhoto(page, 0.8, shots && `${shots}/drone-photo-phone-export.png`, true);
  if (shots) await page.screenshot({ path: `${shots}/drone-photo-phone-mode.png` });
  const d = await droneState(page);
  assert.equal(d.droneFrame, frozen.droneFrame);
  assert.equal(d.droneGenerated, frozen.droneGenerated);
  const receiver = await page.context().newPage();
  try {
    await receiver.goto(`${base}?${d.droneShare}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.dronePhoto === '1' && document.querySelector('canvas')?.dataset.droneSeeking === '0' && Boolean(document.querySelector('canvas')?.dataset.dronePhotoRect));
    const restored = await droneState(receiver);
    for (const key of ['dronePosition', 'droneGhostGates', 'droneFrame', 'dronePhotoOrbit']) assert.equal(restored[key], d[key]);
    assert(new URLSearchParams(restored.droneShare).has('inputs'));
  } finally { await receiver.close(); }
  await page.bringToFront();
  await photoAction(page, 'close', true);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
  const closed = await droneState(page);
  assert.equal(closed.droneReplay, '0');
  assert.equal(closed.dronePosition, frozen.dronePosition);
  assert.equal(closed.droneFrame, frozen.droneFrame);
  await page.keyboard.press('Space');
  await page.keyboard.down('w');
  await page.waitForFunction((frame) => Number(document.querySelector('canvas')?.dataset.droneFrame) > frame, Number(frozen.droneFrame), { timeout: 15000 });
  await page.keyboard.up('w'); await page.keyboard.press('Space');
  assert(Number((await droneState(page)).droneFrame) > Number(frozen.droneFrame), 'manual flight could not resume after a photo');
}

export async function checkPhotoQuery(page) {
  const d = await droneState(page);
  assert.equal(d.dronePhoto, '1');
  assert.equal(d.dronePhotoOrbit, '235.000,20.000,5.000');
  assert.equal(d.dronePhotoFormat, 'wide');
  assert.equal(d.droneFrame, '36');
  assert.equal(d.dronePlaying, '0', 'photo link played despite photo mode');
}

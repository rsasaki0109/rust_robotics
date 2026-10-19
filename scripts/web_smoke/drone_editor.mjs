// Exercise real canvas gestures and shared layouts, without changing Rust state.
import assert from 'node:assert/strict';
import { droneState, waitDrone } from './drone_manual.mjs';

export const coursePoints = (encoded) => encoded.slice(3).split('.').map((row) => row.split('_').map((value) => Number(value) / 10));

export async function editorAction(page, name, touch = false) {
  const d = await droneState(page);
  const position = JSON.parse(d.droneEditorActions)[name];
  if (touch) await page.touchscreen.tap(...position);
  else await page.mouse.click(...position);
}

export async function dragGate(page, index, delta, touch = false) {
  const d = await droneState(page);
  const handles = JSON.parse(d.droneEditorHandles);
  const [, , width, height] = JSON.parse(d.droneEditorMap);
  const start = handles[index];
  const end = [start[0] + delta[0] / 26 * width, start[1] - delta[1] / (d.droneEditorView === 'top' ? 8 : 7) * height];
  if (touch) {
    const cdp = await page.context().newCDPSession(page);
    const point = (pos) => ({ id: 1, x: pos[0], y: pos[1], radiusX: 7, radiusY: 7, force: 1 });
    try {
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [point(start)] });
      for (let i = 1; i <= 8; i++) {
        const pos = start.map((v, axis) => v + (end[axis] - v) * i / 8);
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [point(pos)] });
        await page.waitForTimeout(30);
      }
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    } finally { await cdp.detach(); }
  } else {
    await page.mouse.move(...start); await page.mouse.down();
    await page.waitForTimeout(30);
    for (let i=1;i<=8;i++) {
      await page.mouse.move(...start.map((v,axis)=>v+(end[axis]-v)*i/8));
      await page.waitForTimeout(30);
    }
    await page.mouse.up();
  }
  await page.waitForTimeout(100);
  const after = await droneState(page);
  assert.notEqual(after.droneLayout, d.droneLayout, 'dragging the gate did not change the layout');
  assert.equal(after.droneEditorSelected, String(index));
  assert.equal(after.droneGenerated, '0', 'editing must not generate a flight');
  assert.equal(after.dronePlaying, '0');
  return after;
}

export async function checkEditorDesktop(page, base, shots) {
  const original = await droneState(page);
  await page.mouse.click(...JSON.parse(original.droneEditorOpen));
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneEditor === '1');
  let d = await dragGate(page, 1, [0.4, 0.2]);
  let point = coursePoints(d.droneLayout)[1];
  assert(Math.abs(point[0] - 8.4) < 0.11 && Math.abs(point[1] - 0.2) < 0.11);
  const custom = d.droneLayout;
  await editorAction(page, 'side');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneEditorView === 'side');
  d = await dragGate(page, 1, [0, 0.5]);
  point = coursePoints(d.droneLayout)[1];
  assert(point[2] >= 4.4 && point[2] <= 4.6, 'Side view did not change height');
  await editorAction(page, 'undo');
  await page.waitForTimeout(100);
  assert.equal((await droneState(page)).droneLayout, custom);
  await editorAction(page, 'reset');
  await page.waitForTimeout(100);
  assert.equal((await droneState(page)).droneLayout, original.droneLayout);
  await editorAction(page, 'undo');
  await editorAction(page, 'top');
  await page.waitForTimeout(150);
  d = await droneState(page);
  assert.equal(d.droneLayout, custom);
  assert(new URLSearchParams(d.droneShare).get('gates') === custom);
  if (shots) await page.screenshot({ path: `${shots}/drone-editor-draft.png` });

  const receiver = await page.context().newPage();
  try {
    await receiver.goto(`${base}?${new URLSearchParams(d.droneShare)}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneEditor === '1');
    assert.equal((await droneState(receiver)).droneLayout, custom, 'shared draft lost its gate positions');
    await editorAction(receiver, 'fly');
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneEditor === '0');
    await waitDrone(receiver, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 40);
    await receiver.keyboard.press('Space');
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
    const snapshot = await droneState(receiver);
    assert.equal(snapshot.droneCourse, 'custom');
    const replay = await page.context().newPage();
    try {
      await replay.goto(`${base}?${new URLSearchParams(snapshot.droneShare)}`, { waitUntil: 'load' });
      await waitDrone(replay, () => document.querySelector('canvas')?.dataset.droneSeeking === '0' && Number(document.querySelector('canvas')?.dataset.droneFrame) > 0);
      const pose = await droneState(replay);
      assert.equal(pose.droneLayout, custom);
      assert.equal(pose.dronePosition, snapshot.dronePosition);
      assert.equal(pose.droneFrame, snapshot.droneFrame);
    } finally { await replay.close(); }
    await receiver.bringToFront();
    await receiver.keyboard.press('Space');
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished', 45000);
    assert.equal((await droneState(receiver)).droneGates, '5', 'custom course did not finish by gate crossings');
  } finally { await receiver.close(); }
  await page.bringToFront();
  await editorAction(page, 'cancel');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneEditor === '0');
  const cancelled = await droneState(page);
  assert.equal(cancelled.droneLayout, original.droneLayout, 'Cancel changed the original course');
  assert.equal(cancelled.dronePosition, original.dronePosition, 'editing advanced the paused original attempt');
  assert.equal(cancelled.droneFrame, original.droneFrame);
}

export async function checkEditorPhone(page, base, shots) {
  const original = await droneState(page);
  let d = await dragGate(page, 1, [0.8, 0.5], true);
  await editorAction(page, 'side', true);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneEditorView === 'side');
  d = await dragGate(page, 1, [0, 0.6], true);
  assert(coursePoints(d.droneLayout)[1][2] >= 4.5);
  assert.notEqual(d.droneLayout, original.droneLayout);
  const custom = d.droneLayout;
  if (shots) await page.screenshot({ path: `${shots}/drone-editor-phone-draft.png` });
  await editorAction(page, 'fly', true);
  await waitDrone(page, () => {
    const d=document.querySelector('canvas')?.dataset;
    return d?.droneEditor==='0' && d.dronePad && d.droneActions;
  });
  d = await droneState(page);
  assert.equal(d.droneMode, 'manual');
  assert.equal(d.droneCourse, 'custom');
  assert.equal(d.droneFrame, '0');
  const pad = JSON.parse(d.dronePad);
  const cdp = await page.context().newCDPSession(page);
  const point = (id, pos) => ({ id, x: pos[0], y: pos[1], radiusX: 7, radiusY: 7, force: 1 });
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [point(1, pad.forward), point(2, pad.up)] });
  await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 8);
  await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  await cdp.detach();
  d = await droneState(page);
  await page.touchscreen.tap(...JSON.parse(d.droneActions).pause);
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  d = await droneState(page);
  const receiver = await page.context().newPage();
  try {
    await receiver.goto(`${base}?${new URLSearchParams(d.droneShare)}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneSeeking === '0' && Number(document.querySelector('canvas')?.dataset.droneFrame) > 0);
    const restored = await droneState(receiver);
    assert.equal(restored.droneLayout, custom);
    assert.equal(restored.dronePosition, d.dronePosition);
    assert.equal(restored.droneGhostGates, d.droneGhostGates);
    assert.equal(restored.droneReplay, '1');
    await receiver.touchscreen.tap(...JSON.parse(restored.droneActions).retry);
    await waitDrone(receiver, () => document.querySelector('canvas')?.dataset.droneFrame === '0');
    assert.equal((await droneState(receiver)).droneLayout, custom, 'Retry lost the custom course');
  } finally { await receiver.close(); }
}

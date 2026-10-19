// Limit actual paint scheduling while leaving flight state and inputs alone.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { join } from 'node:path';
import { droneState, waitDrone } from './drone_manual.mjs';

export async function slowReplayFixture(docs) {
  const { finish } = JSON.parse(await readFile(join(docs, 'assets/drone-checkpoint-flight.json'), 'utf8'));
  const query = new URLSearchParams(finish.droneShare);
  for (const [key, value] of Object.entries({ frame: '0', playing: '1', camera: 'chase', slow: '0', edit: '0' })) query.set(key, value);
  query.delete('photo'); query.delete('intro');
  return { query: query.toString(), finish };
}

export async function prepareSlowPaints(page) {
  await page.addInitScript(() => {
    const request = window.requestAnimationFrame.bind(window);
    const cancel = window.cancelAnimationFrame.bind(window);
    const callbacks = new Map();
    let next = 0;
    let timer = null, frame = null, running = false;
    window.__slowPaintTimes = [];
    const schedule = () => {
      if (timer !== null || frame !== null || running || callbacks.size === 0) return;
      timer = window.setTimeout(() => {
        timer = null;
        frame = request((time) => {
          frame = null;
          running = true;
          const times = window.__slowPaintTimes;
          if (times.at(-1) !== time) times.push(time);
          if (times.length > 64) times.shift();
          // Requests made during this batch belong to the next slow frame.
          const batch = [...callbacks];
          for (const [id, callback] of batch) {
            if (callbacks.get(id) !== callback) continue;
            callbacks.delete(id);
            callback.call(window, time);
          }
          running = false;
          schedule();
        });
      }, 400);
    };
    window.requestAnimationFrame = (callback) => {
      const id = ++next;
      callbacks.set(id, callback);
      schedule();
      return id;
    };
    window.cancelAnimationFrame = (id) => {
      callbacks.delete(id);
      if (callbacks.size === 0 && !running) {
        window.clearTimeout(timer);
        timer = null;
        if (frame !== null) cancel(frame);
        frame = null;
      }
    };
  });
}

export async function checkSlowReplay(page, expected) {
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneClearGate === '1', 20000);
  assert.equal((await droneState(page)).droneReplay, '1');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStatus === 'finished', 90000);
  const d = await droneState(page);
  assert.equal(d.droneFrame, expected.droneFrame);
  assert.equal(d.dronePosition, expected.dronePosition);
  assert.equal(d.droneGates, '5');
  assert.equal(d.droneBestSteps, '0', 'a slow shared replay earned a personal best');
  const times = await page.evaluate(() => window.__slowPaintTimes);
  assert(times.length >= 3 && times.slice(1).every((time, i) => time - times[i] >= 350),
    'the browser did not keep the slow paint cadence');
}

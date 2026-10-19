// Observe the real countdown and hold real touch contacts across GO.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { droneState, waitDrone } from './drone_manual.mjs';
import { guideAction } from './drone_v1.mjs';

async function action(page, name) {
  await page.touchscreen.tap(...JSON.parse((await droneState(page)).droneActions)[name]);
}

async function stationary(page, extra = () => true) {
  await waitDrone(page, () => {
    const d = document.querySelector('canvas')?.dataset;
    return d?.droneStart === '' && d.dronePlaying === '0' && d.droneFrame === '0';
  });
  assert(extra(await droneState(page)));
}

export async function checkRaceStart(page, base, shots) {
  const initial = await droneState(page);
  await page.evaluate(() => {
    const canvas = document.querySelector('canvas');
    window.__flightStartTrace = [];
    new MutationObserver(() => {
      const d = { ...canvas.dataset };
      const trace = window.__flightStartTrace;
      if (d.droneStart && trace.at(-1)?.droneStart !== d.droneStart) trace.push(d);
    }).observe(canvas, { attributes: true, attributeFilter: ['data-drone-start'] });
  });
  const cdp = await page.context().newCDPSession(page);
  const pad = JSON.parse(initial.dronePad);
  const point = (id, p) => ({ id, x: p[0], y: p[1], radiusX: 8, radiusY: 8, force: 1 });
  try {
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [point(1, pad.forward), point(2, pad.up)] });
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStart === '3');
    if (shots) await page.screenshot({ path: `${shots}/drone-start-countdown.png` });
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStart === 'go');
    if (shots) await page.screenshot({ path: `${shots}/drone-start-go.png` });
    await waitDrone(page, () => Number(document.querySelector('canvas')?.dataset.droneFrame) >= 8);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
  } finally { await cdp.detach(); }
  await action(page, 'pause');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePlaying === '0');
  const launched = await droneState(page);
  const trace = await page.evaluate(() => window.__flightStartTrace);
  assert.deepEqual(trace.map((d) => d.droneStart), ['3', '2', '1', 'go']);
  for (const d of trace) {
    assert.equal(d.droneFrame, '0', 'countdown advanced the flight clock');
    assert.equal(d.droneGenerated, '0');
    assert.equal(d.droneGhostFrame, '0');
    assert.equal(d.dronePosition, initial.dronePosition);
    // Browsers deliver the first contact before the second. Both contacts
    // must be retained once the pilot has lined them up, including at GO.
    if (d.droneStart !== '3') assert.equal(d.droneInput, '1,0,1', 'held touch input was lost across GO');
    assert(!new URLSearchParams(d.droneShare).has('inputs'));
    if (d.droneStart !== 'go') assert.equal(new URLSearchParams(d.droneShare).get('playing'), '0');
  }
  assert(Number(launched.dronePosition.split(',')[0]) > 0);
  assert(Number(launched.dronePosition.split(',')[2]) > 2.3);
  const inputs = new URLSearchParams(launched.droneShare).get('inputs');
  assert.equal(inputs[3], 'n', 'the first physics command did not retain both touch axes');
  assert.equal(inputs.length - 3, Number(launched.droneFrame), 'countdown time leaked into recorded flight time');

  const receiverContext = await page.context().browser().newContext({
    deviceScaleFactor: await page.evaluate(() => devicePixelRatio),
    viewport: page.viewportSize(),
  });
  const receiver = await receiverContext.newPage();
  await receiver.bringToFront();
  let shared;
  try {
    await receiver.goto(`${base}?${launched.droneShare}`, { waitUntil: 'load' });
    await waitDrone(receiver, () => {
      const d = document.querySelector('canvas')?.dataset;
      return d?.droneSeeking === '0' && Number(d.droneFrame) > 0;
    });
    shared = await droneState(receiver);
    assert.equal(shared.droneStart, '');
    assert.equal(shared.droneReplay, '1');
    assert.equal(shared.dronePosition, launched.dronePosition);
    assert.equal(shared.droneFrame, launched.droneFrame);
    assert.equal(shared.droneBestSteps, '0');
  } finally { await receiverContext.close(); }
  await page.bringToFront();
  if (shots) await writeFile(`${shots}/drone-start-flight.json`, JSON.stringify({ initial, trace, launched, shared }, null, 2) + '\n');
}

export async function checkStartCancellation(page, _base, shots) {
  const start = async () => {
    await action(page, 'pause');
    await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneStart === '3');
  };
  await start();
  await page.keyboard.press('Space');
  await stationary(page);
  await start();
  await page.keyboard.press('h');
  await stationary(page, (d) => d.droneGuide === '1');
  await guideAction(page, 'fly', true);

  await start();
  // Playwright normally forces every page to appear focused. Restore normal
  // browser focus here so this exercises a genuine window/tab transition.
  const sourceSession = await page.context().newCDPSession(page);
  await sourceSession.send('Emulation.setFocusEmulationEnabled', { enabled: false });
  const other = await page.context().newPage();
  const otherSession = await page.context().newCDPSession(other);
  await otherSession.send('Emulation.setFocusEmulationEnabled', { enabled: false });
  await other.bringToFront();
  // A hidden tab may suspend painting entirely. Check cancellation on return,
  // when the application receives the queued loss-of-focus event.
  await other.waitForTimeout(300);
  assert.equal(await page.evaluate(() => document.hasFocus()), false, 'the browser did not actually move focus');
  await otherSession.detach();
  await other.close();
  await page.bringToFront();
  await stationary(page);
  await sourceSession.send('Emulation.setFocusEmulationEnabled', { enabled: true });
  await sourceSession.detach();

  await start();
  await page.touchscreen.tap(...JSON.parse((await droneState(page)).droneMenu));
  await stationary(page, (d) => d.droneMenuOpen === '1');
  await page.keyboard.press('Escape');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneMenuOpen === '0');
  await start();
  await page.keyboard.press('r');
  await stationary(page);
  await start();
  await page.keyboard.press('p');
  await stationary(page, (d) => d.dronePhoto === '1');
  if (shots) await page.screenshot({ path: `${shots}/drone-start-cancelled-photo.png` });
  await page.keyboard.press('Escape');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '0');
  assert.equal((await droneState(page)).droneBestSteps, '0');
}

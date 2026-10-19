// Smoke test for the WASM playground: serves a built `dist/` directory,
// opens every tab (and a phone-sized viewport) in headless Chromium, pokes
// at the scene, and fails on any page error or Rust panic message.
//
// Usage: node web_smoke.mjs <dist-dir> [screenshot-dir]
// Set CHROMIUM_PATH to use a preinstalled Chromium instead of Playwright's.
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { extname, join, normalize } from 'node:path';
import { chromium } from 'playwright';
import { checkManualDesktop, checkManualPhone } from './drone_manual.mjs';

const [dist, shots] = process.argv.slice(2);
if (!dist) {
  console.error('usage: node web_smoke.mjs <dist-dir> [screenshot-dir]');
  process.exit(2);
}

const types = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.wasm': 'application/wasm',
  '.css': 'text/css',
};
const server = createServer(async (request, response) => {
  const path = normalize(decodeURIComponent(new URL(request.url, 'http://x').pathname));
  const file = join(dist, path.endsWith('/') ? `${path}index.html` : path);
  try {
    const body = await readFile(file);
    response.writeHead(200, { 'content-type': types[extname(file)] ?? 'application/octet-stream' });
    response.end(body);
  } catch {
    response.writeHead(404);
    response.end();
  }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
const base = `http://127.0.0.1:${server.address().port}/`;

const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});

// Each case: a share-link query, a viewport, and what to do once loaded.
const cases = [
  { name: 'grid', query: 'tab=grid' },
  { name: 'sampling', query: 'tab=sampling&planner=informed', clickScene: true },
  { name: 'parking', query: 'tab=parking', clickScene: true },
  { name: 'localization', query: 'tab=localization' },
  { name: 'slam-replay', query: 'tab=slam&algorithm=loop' },
  { name: 'drive', query: 'tab=slam&algorithm=drive&world=hall&people=3', clickScene: true },
  { name: 'admm', query: 'tab=admm' },
  { name: 'arena', query: 'tab=arena' },
  { name: 'mppi', query: 'tab=mppi', clickScene: true },
  { name: 'arena-course', query: 'tab=arena&course=5.0,5.0;20.0,5.0;30.0,15.0;45.0,15.0;50.0,25.0' },
  { name: 'pushing', query: 'tab=pushing' },
  { name: 'drone', query: 'tab=drone', drone: 'live' },
  { name: 'drone-replay', query: 'tab=drone&frame=40&playing=0&camera=pilot', drone: 'replay' },
  { name: 'drone-phone', query: 'tab=drone&course=climb&camera=course&frame=30&playing=0', drone: 'phone', viewport: { width: 390, height: 844 }, touch: true },
  { name: 'drone-manual', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'manual' },
  { name: 'drone-manual-phone', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'manual-phone', viewport: { width: 390, height: 844 }, touch: true },
  { name: 'drone-manual-invalid', query: 'tab=drone&mode=manual&cinema=1&inputs=v99.zzz&frame=600&playing=1', drone: 'invalid' },
  { name: 'drive-phone', query: 'tab=slam&algorithm=drive', viewport: { width: 390, height: 844 }, touch: true },
  { name: 'grid-phone', query: 'tab=grid', viewport: { width: 390, height: 844 }, touch: true },
];

let failures = 0;
if (shots) {
  await mkdir(shots, { recursive: true });
}
for (const test of cases) {
  const context = await browser.newContext({
    viewport: test.viewport ?? { width: 1280, height: 900 },
    hasTouch: Boolean(test.touch),
  });
  const page = await context.newPage();
  const problems = [];
  page.on('pageerror', (error) => problems.push(`page error: ${error.message}`));
  page.on('console', (message) => {
    const text = message.text();
    if (message.type() === 'error' || /panicked/.test(text)) {
      problems.push(`console ${message.type()}: ${text.split('\n')[0]}`);
    }
  });
  await page.goto(`${base}?${test.query}`, { waitUntil: 'load' });
  await page.waitForTimeout(4000);
  if (test.drone) {
    try {
      const canvas = '#rust_robotics_playground';
      await page.waitForFunction(() => {
        const c = document.getElementById('rust_robotics_playground');
        return c?.dataset.droneFrame !== undefined && c.dataset.droneSeeking === '0';
      }, null, { timeout: 30000 });
      if (test.drone === 'invalid') {
        const d = await page.locator(canvas).evaluate((c) => ({ ...c.dataset }));
        if (d.droneStatus !== 'error' || d.droneGenerated !== '0' || d.dronePlaying !== '0') {
          problems.push('invalid recording was not rejected before generating flight');
        }
        const actions = JSON.parse(d.droneActions);
        await page.mouse.click(...actions.retry);
        await page.waitForFunction(() => document.querySelector('canvas')?.dataset.droneStatus === 'flying');
        if (await page.locator(canvas).getAttribute('data-drone-frame') !== '0') {
          problems.push('retry did not recover a fresh manual attempt');
        }
      } else if (test.drone === 'manual') {
        await checkManualDesktop(page, base, shots);
      } else if (test.drone === 'manual-phone') {
        await checkManualPhone(page);
      } else if (test.drone === 'live') {
        await page.waitForFunction(() => document.getElementById('rust_robotics_playground')?.dataset.droneStatus === 'finished', null, { timeout: 45000 });
        if (await page.locator(canvas).getAttribute('data-drone-gates') !== '5') {
          problems.push('the drone did not pass all five gates');
        }
        // Exercise the real canvas keyboard controls and cached replay.
        await page.mouse.click(900, 380);
        await page.keyboard.press('r');
        await page.waitForFunction(() => {
          const d = document.getElementById('rust_robotics_playground')?.dataset;
          return d?.dronePlaying === '1' && Number(d.droneFrame) < 10;
        });
        await page.keyboard.press('2');
        await page.waitForFunction(() => document.getElementById('rust_robotics_playground')?.dataset.droneCamera === 'pilot');
        await page.keyboard.press('Space');
        await page.waitForFunction(() => document.getElementById('rust_robotics_playground')?.dataset.dronePlaying === '0');
        const frame = await page.locator(canvas).getAttribute('data-drone-frame');
        await page.waitForTimeout(250);
        if (await page.locator(canvas).getAttribute('data-drone-frame') !== frame) {
          problems.push('paused drone replay continued to advance');
        }
      } else {
        const expected = test.drone === 'phone' ? '30' : '40';
        const frame = await page.locator(canvas).getAttribute('data-drone-frame');
        const position = await page.locator(canvas).getAttribute('data-drone-position');
        if (frame !== expected) problems.push(`shared drone frame is ${frame}, expected ${expected}`);
        await page.waitForTimeout(250);
        if (await page.locator(canvas).getAttribute('data-drone-position') !== position) {
          problems.push('shared paused drone pose changed');
        }
      }
    } catch (error) {
      problems.push(`drone functional check: ${error.message.split('\n')[0]}`);
    }
  }
  if (test.clickScene) {
    // Send the robot somewhere (the scene fills the right of the side
    // panel): exercises planning and the navigator.
    await page.mouse.move(800, 420);
    await page.mouse.click(800, 420);
    await page.waitForTimeout(3000);
  }
  const loading = await page.evaluate(() => Boolean(document.getElementById('loading')));
  if (loading) {
    problems.push('the loading placeholder never went away');
  }
  if (shots) {
    await page.screenshot({ path: join(shots, `${test.name}.png`) });
  }
  console.log(`${problems.length ? 'FAIL' : 'ok  '} ${test.name}`);
  for (const problem of problems) {
    console.log(`     ${problem}`);
  }
  failures += problems.length ? 1 : 0;
  await context.close();
}

await browser.close();
server.close();
if (failures) {
  console.error(`${failures} of ${cases.length} cases failed`);
  process.exit(1);
}

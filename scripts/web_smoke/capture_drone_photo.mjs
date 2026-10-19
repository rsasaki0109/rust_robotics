// Export real PNGs through the playground UI and save their replayable receipt.
// Usage: node capture_drone_photo.mjs <dist-dir> <out-dir>
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { mkdir, readFile, writeFile } from 'node:fs/promises';
import { extname, join, resolve } from 'node:path';
import { chromium } from 'playwright';
import { droneState, waitDrone } from './drone_manual.mjs';
import { orbitPhoto, photoAction, savePhoto } from './drone_photo.mjs';

const [distArg, outArg] = process.argv.slice(2);
if (!distArg || !outArg) throw new Error('usage: node capture_drone_photo.mjs <dist-dir> <out-dir>');
const dist = resolve(distArg), out = resolve(outArg);
const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm' };
const server = createServer(async (request, response) => {
  const path = decodeURIComponent(new URL(request.url, 'http://x').pathname);
  const file = resolve(dist, `.${path.endsWith('/') ? `${path}index.html` : path}`);
  if (!file.startsWith(`${dist}/`)) { response.writeHead(403).end(); return; }
  try {
    response.writeHead(200, { 'content-type': types[extname(file)] ?? 'application/octet-stream' });
    response.end(await readFile(file));
  } catch { response.writeHead(404).end(); }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
await mkdir(out, { recursive: true });
const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const base = `http://127.0.0.1:${server.address().port}/`;
const context = await browser.newContext({ viewport: { width: 1600, height: 1100 } });
const page = await context.newPage();
const problems = [];
page.on('pageerror', (error) => problems.push(error.message));
page.on('console', (message) => {
  if (message.type() === 'error' || /panicked/.test(message.text())) problems.push(message.text());
});
try {
  await page.goto(`${base}?tab=drone&course=climb&frame=45&playing=0&cinema=1`, { waitUntil: 'load' });
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.droneFrame === '45' && document.querySelector('canvas')?.dataset.droneSeeking === '0');
  const original = await droneState(page);
  await page.keyboard.press('p');
  await waitDrone(page, () => document.querySelector('canvas')?.dataset.dronePhoto === '1');
  await orbitPhoto(page);
  await photoAction(page, 'in');
  await photoAction(page, 'in');
  const photos = [];
  for (const [format, ratio] of [['wide', 16 / 9], ['square', 1], ['portrait', 0.8]]) {
    await photoAction(page, format);
    const snapshot = await droneState(page);
    const image = await savePhoto(page, ratio, join(out, `${format}.png`));
    assert.equal(snapshot.dronePosition, original.dronePosition);
    photos.push({ format, ...image, state: snapshot });
    if (format === 'wide') await page.screenshot({ path: join(out, 'photo-mode.png') });
  }
  assert.deepEqual(problems, []);
  await writeFile(join(out, 'capture.json'), JSON.stringify({ original, photos, problems }, null, 2) + '\n');
  console.log(JSON.stringify(photos.map(({ state, ...image }) => image)));
} finally {
  await browser.close();
  server.close();
}

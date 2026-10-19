// Capture a real 30-second browser flight, including all three cameras.
// Usage: node capture_drone_race.mjs <dist-dir> <out-dir>
// Requires FFmpeg with libx264 (or set FFMPEG_PATH). Writes flight.mp4.
import { createServer } from 'node:http';
import { readFile, mkdir, writeFile } from 'node:fs/promises';
import { spawn, spawnSync } from 'node:child_process';
import { once } from 'node:events';
import { extname, join, resolve } from 'node:path';
import { chromium } from 'playwright';

const [distArg, outArg] = process.argv.slice(2);
if (!distArg || !outArg) throw new Error('usage: capture_drone_race.mjs <dist-dir> <out-dir>');
const [dist, out] = [resolve(distArg), resolve(outArg)];
const ffmpeg = process.env.FFMPEG_PATH || 'ffmpeg';
if (spawnSync(ffmpeg, ['-version'], { stdio: 'ignore' }).status !== 0) {
  throw new Error('Install FFmpeg, or set FFMPEG_PATH to its executable');
}
const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm' };
const server = createServer(async (request, response) => {
  const path = decodeURIComponent(new URL(request.url, 'http://x').pathname);
  const file = resolve(dist, `.${path.endsWith('/') ? `${path}index.html` : path}`);
  if (!file.startsWith(`${dist}/`)) { response.writeHead(403).end(); return; }
  try {
    const body = await readFile(file);
    response.writeHead(200, { 'content-type': types[extname(file)] ?? 'application/octet-stream' });
    response.end(body);
  } catch { response.writeHead(404).end(); }
});
await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
const base = `http://127.0.0.1:${server.address().port}/`;
await mkdir(out, { recursive: true });
const browser = await chromium.launch({
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const problems = [];
const context = await browser.newContext({
  viewport: { width: 1280, height: 760 },
});
const page = await context.newPage();
page.on('pageerror', (error) => problems.push(error.message));
page.on('console', (message) => {
  if (message.type() === 'error' || /panicked/.test(message.text())) problems.push(message.text());
});
let stopRecording;
try {
  await page.goto(`${base}?tab=drone&playing=0&slow=1&cinema=1`, { waitUntil: 'load' });
  await page.waitForFunction(() => document.getElementById('rust_robotics_playground')?.dataset.droneFrame === '0', null, { timeout: 30000 });
  // CDP provides the actual browser frames and timestamps. Repeat the previous
  // frame when the app is idle, preserving wall-clock timing in a 25fps movie.
  // Using system FFmpeg also works with a preinstalled system Chromium.
  const session = await context.newCDPSession(page);
  const encoder = spawn(ffmpeg, ['-y', '-f', 'image2pipe', '-vcodec', 'mjpeg',
    '-framerate', '25', '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-threads', '2',
    '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p', '-movflags', '+faststart',
    join(out, 'flight.mp4')], { stdio: ['pipe', 'ignore', 'pipe'] });
  let encoderLog = '';
  encoder.stderr.on('data', (chunk) => { encoderLog = (encoderLog + chunk).slice(-12000); });
  const encoded = new Promise((resolve, reject) => {
    encoder.on('error', reject);
    encoder.on('exit', (code) => code === 0 ? resolve() : reject(new Error(`FFmpeg exited ${code}: ${encoderLog}`)));
  });
  // Attach a handler immediately; the final await still propagates errors.
  encoded.catch(() => {});
  let firstTimestamp, firstWall, lastFrame, frames = 0, accepting = true;
  let pending = Promise.resolve();
  const append = (image, count) => {
    pending = pending.then(async () => {
      for (let i = 0; i < count; i++) {
        if (!encoder.stdin.write(image)) await once(encoder.stdin, 'drain');
      }
    });
    pending.catch(() => {});
  };
  session.on('Page.screencastFrame', ({ data, metadata, sessionId }) => {
    session.send('Page.screencastFrameAck', { sessionId }).catch(() => {});
    if (!accepting) return;
    const image = Buffer.from(data, 'base64');
    if (firstTimestamp === undefined) {
      firstTimestamp = metadata.timestamp;
      firstWall = Date.now();
      lastFrame = image;
    }
    const target = Math.min(750, Math.max(1, Math.round((metadata.timestamp - firstTimestamp) * 25)));
    append(lastFrame, Math.max(0, target - frames));
    frames = Math.max(frames, target);
    lastFrame = image;
  });
  await session.send('Page.startScreencast', { format: 'jpeg', quality: 90, maxWidth: 1280, maxHeight: 760, everyNthFrame: 1 });
  stopRecording = async () => {
    accepting = false;
    await session.send('Page.stopScreencast');
    if (lastFrame) {
      const target = Math.min(750, Math.ceil((Date.now() - firstWall) / 1000 * 25));
      append(lastFrame, Math.max(0, target - frames));
    }
    await pending;
    encoder.stdin.end();
    await encoded;
    await writeFile(join(out, 'encoder.log'), encoderLog);
  };
  await page.mouse.click(900, 380);
  const started = Date.now();
  await page.keyboard.press('Space');
  await page.waitForTimeout(4500);
  await page.screenshot({ path: join(out, 'chase.png') });
  await page.keyboard.press('2');
  await page.waitForTimeout(4000);
  await page.screenshot({ path: join(out, 'pilot.png') });
  await page.keyboard.press('3');
  await page.waitForFunction(() => document.getElementById('rust_robotics_playground')?.dataset.droneStatus === 'finished', null, { timeout: 45000 });
  const firstFlight = await page.locator('canvas').evaluate((c) => ({ ...c.dataset }));
  if (firstFlight.droneGates !== '5') throw new Error('recorded flight did not finish');
  await page.waitForTimeout(1500);
  await page.keyboard.press('r');
  await page.waitForTimeout(3500);
  await page.screenshot({ path: join(out, 'course.png') });
  await page.keyboard.press('1');
  await page.waitForTimeout(4500);
  await page.keyboard.press('2');
  await page.waitForTimeout(4500);
  await page.screenshot({ path: join(out, 'finish.png') });
  await page.waitForTimeout(Math.max(0, 31000 - (Date.now() - started)));
  await writeFile(join(out, 'capture.json'), JSON.stringify({ firstFlight, problems }, null, 2) + '\n');
} finally {
  try { if (stopRecording) await stopRecording(); }
  finally {
  await context.close();
  await browser.close();
  server.close();
  }
}
if (problems.length) throw new Error(problems.join('\n'));
console.log(`Captured flight and three camera views in ${out}`);

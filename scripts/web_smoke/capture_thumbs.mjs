// Captures the playground screenshots used by the GitHub Pages landing page
// (docs/assets/playground/*.png) from a built playground `dist/` directory.
//
// Usage: node capture_thumbs.mjs <dist-dir> <out-dir> [name...]
// Set CHROMIUM_PATH to use a preinstalled Chromium instead of Playwright's.
//
// egui draws into a canvas, so the few interactions here (a navigation goal
// click) use positions in the default layout at the viewport size below;
// everything else is set up through share-link queries.
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { extname, join, normalize } from 'node:path';
import { chromium } from 'playwright';

const [dist, out, ...only] = process.argv.slice(2);
if (!dist || !out) {
  console.error('usage: node capture_thumbs.mjs <dist-dir> <out-dir> [name...]');
  process.exit(2);
}

// Grid Planners map for the share link: border, the default center wall,
// and a few more walls so the path has to find its way around.
function gridMap() {
  const [w, h] = [32, 24];
  const wall = new Set();
  const add = (x, y) => wall.add(`${x},${y}`);
  for (let x = 0; x < w; x++) { add(x, 0); add(x, h - 1); }
  for (let y = 0; y < h; y++) { add(0, y); add(w - 1, y); }
  for (let y = 6; y <= 17; y++) if (y !== 12) add(16, y);
  for (let y = 2; y <= 17; y++) add(22, y);
  for (let y = 8; y <= 21; y++) add(8, y);
  for (let x = 8; x <= 12; x++) add(x, 8);
  for (let y = 14; y <= 21; y++) add(26, y);
  let hex = '';
  for (let group = 0; group < (w * h) / 4; group++) {
    let nibble = 0;
    for (let bit = 0; bit < 4; bit++) {
      const index = group * 4 + bit;
      if (wall.has(`${index % w},${Math.floor(index / w)}`)) nibble |= 1 << bit;
    }
    hex += nibble.toString(16);
  }
  return hex;
}

// name, share query, settle time [ms], action, viewport, clip to the scene
const shots = [
  { name: 'hero', query: 'tab=slam&algorithm=drive&auto=1', wait: 110000, size: [1280, 760] },
  { name: 'drive', query: 'tab=slam&algorithm=drive&world=hall&people=3', wait: 9000, goal: [930, 160] },
  { name: 'grid', query: `tab=grid&planner=astar&start=2,12&goal=29,12&map=${gridMap()}`, wait: 1500 },
  { name: 'sampling', query: 'tab=sampling', wait: 6000 },
  { name: 'parking', query: 'tab=parking', wait: 3500 },
  { name: 'localization', query: 'tab=localization', wait: 9000 },
  { name: 'arena', query: 'tab=arena', wait: 9000 },
  { name: 'pushing', query: 'tab=pushing', wait: 600 },
  { name: 'admm', query: 'tab=admm', wait: 2600 },
];

const types = { '.html': 'text/html', '.js': 'text/javascript', '.wasm': 'application/wasm', '.css': 'text/css' };
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
await mkdir(out, { recursive: true });
for (const shot of shots.filter((s) => !only.length || only.includes(s.name))) {
  const [width, height] = shot.size ?? [1040, 640];
  const page = await browser.newPage({ viewport: { width, height } });
  await page.goto(`${base}?${shot.query}`, { waitUntil: 'load' });
  await page.waitForTimeout(3000);
  if (shot.goal) {
    await page.mouse.click(...shot.goal);
  }
  await page.waitForTimeout(shot.wait);
  const options = { path: join(out, `${shot.name}.png`) };
  if (!shot.size) {
    // Just the scene, right of the side panel.
    options.clip = { x: 330, y: 50, width: width - 340, height: height - 60 };
  }
  await page.screenshot(options);
  console.log(`captured ${shot.name}`);
  await page.close();
}
await browser.close();
server.close();

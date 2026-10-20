// Smoke test for the WASM playground: serves a built `dist/` directory,
// opens every tab (and a phone-sized viewport) in Chromium, pokes
// at the scene, and fails on any page error or Rust panic message.
//
// Build with --public-url /rust_robotics/playground/, as Pages does.
// Usage: node web_smoke.mjs <dist-dir> [screenshot-dir] [docs-dir]
// Set CHROMIUM_PATH to use a preinstalled Chromium instead of Playwright's.
// The focus-loss case needs a display; on Linux CI use xvfb-run -a node ...
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { extname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { checkContactLab } from './contact_lab.mjs';
import { checkRrtReplay } from './rrt_replay.mjs';
import { checkArm } from './arm.mjs';
import { checkArmTracking } from './arm_tracking.mjs';
import { checkSlamLoop } from './slam_loop.mjs';
import { checkFormationLab } from './formation_lab.mjs';
import { checkMppiInspector } from './mppi_inspector.mjs';
import { checkManualDesktop, checkManualPhone } from './drone_manual.mjs';
import { checkEditorDesktop, checkEditorPhone } from './drone_editor.mjs';
import { checkPhotoDesktop, checkPhotoPhone, checkPhotoQuery } from './drone_photo.mjs';
import { openFromShowcase, checkFlightV1, checkPhoneV1 } from './drone_v1.mjs';
import { checkPersonalBest, checkSessionBest } from './drone_records.mjs';
import { checkBestGhost } from './drone_ghost.mjs';
import { checkRaceStart, checkStartCancellation } from './drone_start.mjs';
import { checkGateFeedback, checkGateFeedbackPhone } from './drone_checkpoint.mjs';
import { prepareSlowPaints, slowReplayFixture, checkSlowReplay } from './drone_slow.mjs';
import { checkShowcase } from './showcase.mjs';
import { checkGridSearch, checkGridScroll } from './grid_search.mjs';
import { checkLocalization } from './localization.mjs';
import { checkControllerInspector, checkControllerScroll } from './controller_inspector.mjs';

const [distArg, shots, docsArg] = process.argv.slice(2);
if (!distArg) {
  console.error('usage: node web_smoke.mjs <dist-dir> [screenshot-dir] [docs-dir]');
  process.exit(2);
}
const dist = resolve(distArg);
const docs = resolve(docsArg ?? fileURLToPath(new URL('../../docs/', import.meta.url)));
const projectPrefix = '/rust_robotics/';
const playgroundPrefix = `${projectPrefix}playground/`;

const types = {
  '.html': 'text/html',
  '.js': 'text/javascript',
  '.wasm': 'application/wasm',
  '.css': 'text/css',
  '.png': 'image/png',
  '.svg': 'image/svg+xml',
  '.gif': 'image/gif',
};
const server = createServer(async (request, response) => {
  const path = decodeURIComponent(new URL(request.url, 'http://x').pathname);
  if (!path.startsWith(projectPrefix)) { response.writeHead(404).end(); return; }
  let root, relative;
  if (path.startsWith(playgroundPrefix)) {
    root = dist; relative = path.slice(playgroundPrefix.length);
  } else if (path.startsWith(`${projectPrefix}img/`)) {
    root = resolve(docs, '../img'); relative = path.slice(`${projectPrefix}img/`.length);
  } else {
    root = docs; relative = path.slice(projectPrefix.length);
  }
  const file = resolve(root, relative.endsWith('/') || !relative ? `${relative}index.html` : relative);
  if (!file.startsWith(`${root}/`)) { response.writeHead(403).end(); return; }
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
const origin = `http://127.0.0.1:${server.address().port}`;
const base = `${origin}${playgroundPrefix}`;
const showcase = `${origin}${projectPrefix}`;

const launch = (scale = 1, headed = false) => chromium.launch({
  channel: 'chromium',
  headless: !headed,
  executablePath: process.env.CHROMIUM_PATH || undefined,
  args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist', `--force-device-scale-factor=${scale}`],
});
const browser = await launch();

// Each case: a share-link query, a viewport, and what to do once loaded.
const slowReplay = await slowReplayFixture(docs);
const cases = [
  { name: 'mppi-inspector', query: 'tab=mppi&mppi_inspect=1&mppi_case=circle&mppi_at=0', mppiInspector: true },
  { name: 'mppi-inspector-phone', query: 'tab=mppi&mppi_inspect=1&mppi_case=circle&mppi_at=0', mppiInspector: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'mppi-inspector-compact', query: 'tab=mppi&mppi_inspect=1&mppi_case=circle&mppi_at=0', mppiInspector: true, viewport: { width: 900, height: 700 } },
  { name: 'mppi-inspector-slow', query: 'tab=mppi&mppi_inspect=1&mppi_case=circle&mppi_at=0', mppiInspector: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'formation-lab', query: 'tab=admm&formation_view=inspect&frame=0&playing=0', formationLab: true },
  { name: 'formation-lab-phone', query: 'tab=admm&formation_view=inspect&frame=0&playing=0', formationLab: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'formation-lab-compact', query: 'tab=admm&formation_view=inspect&frame=0&playing=0', formationLab: true, viewport: { width: 900, height: 700 } },
  { name: 'formation-lab-slow', query: 'tab=admm&formation_view=inspect&frame=0&playing=0', formationLab: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'slam-loop', query: 'tab=slam&algorithm=loop&loop_view=compare&frame=0&playing=0', slamLoop: true },
  { name: 'slam-loop-phone', query: 'tab=slam&algorithm=loop&loop_view=compare&frame=0&playing=0', slamLoop: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'slam-loop-slow', query: 'tab=slam&algorithm=loop&loop_view=compare&frame=0&playing=0', slamLoop: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'arm-tracking', query: 'tab=arm&arm_view=compare', armTracking: true },
  { name: 'arm-tracking-phone', query: 'tab=arm&arm_view=compare', armTracking: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'arm-tracking-slow', query: 'tab=arm&arm_view=compare', armTracking: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'arm', query: 'tab=arm', arm: true },
  { name: 'arm-phone', query: 'tab=arm', arm: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'arm-slow', query: 'tab=arm', arm: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'rrt-replay', query: 'tab=sampling&planner=rrtstar&seed=7&rrt_frame=0&rrt_playing=0', rrtReplay: true },
  { name: 'rrt-replay-phone', query: 'tab=sampling&planner=rrtstar&seed=7&rrt_frame=0&rrt_playing=0', rrtReplay: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'rrt-replay-slow', query: 'tab=sampling&planner=rrtstar&seed=7&rrt_frame=0&rrt_playing=0', rrtReplay: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'contact-lab', query: 'tab=pushing&push_mode=contact', contactLab: true },
  { name: 'contact-lab-phone', query: 'tab=pushing&push_mode=contact', contactLab: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'contact-lab-slow', query: 'tab=pushing&push_mode=contact', contactLab: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'grid-slow-input', query: 'tab=grid&planner=astar&search_rate=5', gridSearch: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'controller-inspector-slow-scroll', query: 'tab=arena&preset=hairpin&arena_frame=100&arena_playing=0&arena_focus=stanley', controllerInspector: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'controller-inspector', query: 'tab=arena&preset=hairpin&arena_frame=100&arena_playing=0&arena_focus=stanley', controllerInspector: true },
  { name: 'controller-inspector-phone', query: 'tab=arena&preset=hairpin&arena_frame=100&arena_playing=0&arena_focus=stanley', controllerInspector: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'localization-slow-input', query: 'tab=localization&loc_playing=0&loc_tour=0', localization: true, slowPaint: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'localization-observations', query: 'tab=localization&loc_playing=0&loc_tour=0', localization: true },
  { name: 'localization-observations-phone', query: 'tab=localization&loc_playing=0&loc_tour=0', localization: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'grid-search', query: 'tab=grid&planner=astar&search_rate=5', gridSearch: true },
  { name: 'grid-search-phone', query: 'tab=grid&planner=astar&search_rate=5', gridSearch: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'showcase-explore', showcase: true },
  { name: 'showcase-explore-phone', showcase: true, viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'drone-slow-replay', query: slowReplay.query, drone: 'slow-replay', slowPaint: true, expected: slowReplay.finish },
  { name: 'drone-gate-feedback', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'gate-feedback' },
  { name: 'drone-gate-feedback-phone', query: 'tab=drone&course=climb&cinema=1&frame=0&playing=0&slow=1', drone: 'gate-feedback-phone', viewport: { width: 390, height: 844 }, touch: true, scale: 2 },
  { name: 'drone-race-start', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'race-start', viewport: { width: 390, height: 844 }, touch: true, scale: 2 },
  // Headless Chromium versions differ in whether bringToFront changes focus.
  // Exercise a real browser window for the focus-loss assertion.
  { name: 'drone-start-cancel', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'start-cancel', viewport: { width: 844, height: 390 }, touch: true, headed: true },
  { name: 'drone-best-ghost', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'best-ghost', viewport: { width: 390, height: 844 }, touch: true, scale: 2 },
  { name: 'drone-best-ghost-invalid', query: 'tab=drone&course=climb&mode=manual&cinema=1&rival=best&best_inputs=v1.n&frame=600&playing=1', drone: 'invalid' },
  { name: 'drone-records', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'records', viewport: { width: 390, height: 844 }, touch: true, scale: 2 },
  { name: 'drone-records-session', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'records-session', viewport: { width: 390, height: 844 }, touch: true, storageBlocked: true },
  { name: 'drone-v1', home: true, drone: 'v1' },
  { name: 'drone-v1-phone', home: true, drone: 'v1-phone', viewport: { width: 320, height: 740 }, touch: true, scale: 2 },
  { name: 'drone-v1-landscape', home: true, drone: 'v1-landscape', viewport: { width: 844, height: 390 }, touch: true },
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
  { name: 'drone-editor', query: 'tab=drone&course=climb&cinema=1&frame=30&playing=0', drone: 'editor' },
  { name: 'drone-editor-phone', query: 'tab=drone&course=climb&mode=manual&cinema=1&edit=1', drone: 'editor-phone', viewport: { width: 390, height: 844 }, touch: true },
  { name: 'drone-course-invalid', query: 'tab=drone&course=custom&gates=v1.40_0_28&cinema=1&frame=600&playing=1&edit=1', drone: 'course-invalid' },
  { name: 'drone-photo', query: 'tab=drone&course=custom&gates=v1.40_0_28.84_2_40.120_8_52.160_-6_56.200_0_40&cinema=1&frame=45&playing=0', drone: 'photo' },
  { name: 'drone-photo-phone', query: 'tab=drone&course=climb&mode=manual&cinema=1', drone: 'photo-phone', viewport: { width: 390, height: 844 }, touch: true, scale: 2 },
  { name: 'drone-photo-query', query: 'tab=drone&photo=1&frame=36&yaw=NaN&pitch=100&distance=0&format=bad&playing=1', drone: 'photo-query' },
  { name: 'drive-phone', query: 'tab=slam&algorithm=drive', viewport: { width: 390, height: 844 }, touch: true },
  { name: 'grid-phone', query: 'tab=grid', viewport: { width: 390, height: 844 }, touch: true },
];

let failures = 0;
if (shots) {
  await mkdir(shots, { recursive: true });
}
for (const test of cases) {
  // Chromium's context-only DPR emulation leaves ResizeObserver's physical
  // box at the host scale. Match the actual display too, as eframe uses that
  // box to size its framebuffer. Otherwise this tests an inconsistent device.
  const caseBrowser = test.scale > 1 || test.headed
    ? await launch(test.scale ?? 1, Boolean(test.headed)) : browser;
  const context = await caseBrowser.newContext({
    viewport: test.viewport ?? { width: 1280, height: 900 },
    hasTouch: Boolean(test.touch),
    deviceScaleFactor: test.scale ?? 1,
  });
  if (test.storageBlocked) await context.addInitScript(() => {
    Object.defineProperty(window, 'localStorage', { get() { throw new DOMException('Storage is disabled', 'SecurityError'); } });
  });
  const page = await context.newPage();
  if (test.slowPaint) await prepareSlowPaints(page);
  const problems = [];
  page.on('pageerror', (error) => problems.push(`page error: ${error.message}`));
  page.on('console', (message) => {
    const text = message.text();
    if (message.type() === 'error' || /panicked/.test(text)) {
      problems.push(`console ${message.type()}: ${text.split('\n')[0]}`);
    }
  });
  if (test.showcase) {
    await page.route('https://fonts.googleapis.com/**', (route) => route.fulfill({ contentType: 'text/css', body: '' }));
    await page.goto(showcase, { waitUntil: 'load' });
  } else if (test.home) await openFromShowcase(page, showcase, shots, test.name);
  else await page.goto(`${base}?${test.query}`, { waitUntil: 'load' });
  await page.waitForTimeout(4000);
  if (test.mppiInspector) {
    try { await checkMppiInspector(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`MPPI: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('mppi') && !['mppiCloud','mppiSelected','mppiNominal','mppiWitnesses'].includes(key))))}`);
    }
  }
  if (test.formationLab) {
    try { await checkFormationLab(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`formation: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('formation') && !['formationRuns','formationWitnesses'].includes(key))))}`);
    }
  }
  if (test.slamLoop) {
    try { await checkSlamLoop(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`SLAM loop: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('loop') && !['loopErrors','loopEvents','loopNodes','loopFrontNodes','loopNodeTruth','loopShifts'].includes(key))))}`);
    }
  }
  if (test.armTracking) {
    try { await checkArmTracking(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`arm tracking: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('armCmp') && key!=='armCmpErrors')))}`);
    }
  }
  if (test.arm) {
    try { await checkArm(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`arm: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('arm'))))}`);
    }
  }
  if (test.rrtReplay) {
    try { await checkRrtReplay(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d=await page.locator('#rust_robotics_playground').evaluate((c)=>({...c.dataset}));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`RRT input: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key])=>key.startsWith('rrt') && !['rrtNodes','rrtCosts'].includes(key))))}`);
    }
  }
  if (test.contactLab) {
    try { await checkContactLab(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d = await page.locator('#rust_robotics_playground').evaluate((c) => ({ ...c.dataset }));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`contact lab: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key]) => key.startsWith('lab'))))}`);
    }
  }
  if (test.controllerInspector) {
    try {
      if (test.slowPaint) await checkControllerScroll(page, shots);
      else await checkControllerInspector(page, base, shots, Boolean(test.touch));
    }
    catch (error) {
      const d = await page.locator('#rust_robotics_playground').evaluate((c) => ({ ...c.dataset }));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`arena input: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key]) => ['arenaActionCount', 'arenaLastAction', 'arenaUiRevision', 'arenaPlaying', 'arenaFrame', 'arenaActions', 'arenaHits', 'arenaPointer'].includes(key))))}`);
    }
  }
  if (test.localization) {
    try { await checkLocalization(page, base, shots, Boolean(test.touch), Boolean(test.slowPaint)); }
    catch (error) {
      const d = await page.locator('#rust_robotics_playground').evaluate((c) => ({ ...c.dataset }));
      problems.push(`${test.name}: ${error.stack ?? error.message}`);
      problems.push(`localization input: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key]) => ['locActionCount', 'locLastAction', 'locUiRevision', 'locPlaying', 'locSteps', 'locActions'].includes(key))))}`);
    }
  }
  if (test.gridSearch) {
    try {
      if (test.slowPaint) await checkGridScroll(page, shots);
      else await checkGridSearch(page, base, shots, Boolean(test.touch));
    }
    catch (error) {
      problems.push(`grid search check: ${error.stack ?? error.message}`);
      const d = await page.locator('#rust_robotics_playground').evaluate((c) => ({ ...c.dataset }));
      problems.push(`grid state: ${JSON.stringify(Object.fromEntries(Object.entries(d).filter(([key]) => key.startsWith('grid') && !['gridExpanded', 'gridShare'].includes(key))))}`);
    }
  }
  if (test.showcase) {
    try { await checkShowcase(page, showcase, shots, test.name); }
    catch (error) { problems.push(`showcase functional check: ${error.stack ?? error.message}`); }
  }
  if (test.drone) {
    try {
      const canvas = '#rust_robotics_playground';
      await page.waitForFunction(() => {
        const c = document.getElementById('rust_robotics_playground');
        return c?.dataset.droneFrame !== undefined && c.dataset.droneSeeking === '0';
      }, null, { timeout: 30000 });
      if (test.drone === 'slow-replay') {
        await checkSlowReplay(page, test.expected);
      } else if (test.drone === 'gate-feedback') {
        await checkGateFeedback(page, base, shots);
      } else if (test.drone === 'gate-feedback-phone') {
        await checkGateFeedbackPhone(page, base, shots);
      } else if (test.drone === 'race-start') {
        await checkRaceStart(page, base, shots);
      } else if (test.drone === 'start-cancel') {
        await checkStartCancellation(page, base, shots);
      } else if (test.drone === 'best-ghost') {
        await checkBestGhost(page, base, shots);
      } else if (test.drone === 'records') {
        await checkPersonalBest(page, base, shots);
      } else if (test.drone === 'records-session') {
        await checkSessionBest(page, base, shots);
      } else if (test.drone === 'v1') {
        await checkFlightV1(page, base, shots);
      } else if (test.drone === 'v1-phone' || test.drone === 'v1-landscape') {
        await checkPhoneV1(page, shots, test.name);
      } else if (test.drone === 'photo') {
        await checkPhotoDesktop(page, base, shots);
      } else if (test.drone === 'photo-phone') {
        await checkPhotoPhone(page, base, shots);
      } else if (test.drone === 'photo-query') {
        await checkPhotoQuery(page);
      } else if (test.drone === 'editor') {
        await checkEditorDesktop(page, base, shots);
      } else if (test.drone === 'editor-phone') {
        await checkEditorPhone(page, base, shots);
      } else if (test.drone === 'course-invalid') {
        const d = await page.locator(canvas).evaluate((c) => ({ ...c.dataset }));
        if (d.droneStatus !== 'error' || d.droneGenerated !== '0' || d.droneEditor !== '0') {
          problems.push('invalid custom course was not rejected before flight or editing');
        }
        await page.mouse.click(800, 350);
        await page.keyboard.press('r');
        await page.waitForFunction(() => document.querySelector('canvas')?.dataset.droneStatus === 'flying');
      } else if (test.drone === 'invalid') {
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
      problems.push(`drone functional check: ${error.stack ?? error.message}`);
      // Keep enough state to distinguish a paused flight, slow painting and
      // unfinished seeking when only the public job annotations are visible.
      const fields = ['frame', 'generated', 'playing', 'status', 'gates', 'mode', 'replay',
        'seeking', 'start', 'clear-gate', 'clear-age', 'guide', 'photo', 'editor', 'menu-open'];
      const values = await Promise.all(fields.map((field) => page.locator('#rust_robotics_playground')
        .getAttribute(`data-drone-${field}`, { timeout: 2000 }).catch(() => null)));
      problems.push(`flight state: ${JSON.stringify(Object.fromEntries(fields.map((field, i) => [field, values[i]])))}`);
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
  if (problems.length && process.env.GITHUB_ACTIONS === 'true') {
    // Keep the failing case and stack visible in the job's annotations,
    // including when the viewer cannot open the authenticated log stream.
    const detail = problems.join('\n').replaceAll('%', '%25').replaceAll('\r', '%0D').replaceAll('\n', '%0A');
    console.log(`::error title=Browser smoke ${test.name}::${detail}`);
  }
  failures += problems.length ? 1 : 0;
  await context.close();
  if (caseBrowser !== browser) await caseBrowser.close();
}

await browser.close();
server.close();
if (failures) {
  console.error(`${failures} of ${cases.length} cases failed`);
  process.exit(1);
}

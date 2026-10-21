// Start on the actual Pages project path, choose a topic, and enter its WASM demo.
import assert from 'node:assert/strict';
import { join } from 'node:path';

const topics = [
  { id: 'planning', label: 'Planning', count: 3, tab: 'parking' },
  { id: 'perception', label: 'Localization & SLAM', count: 2, tab: 'slam' },
  { id: 'control', label: 'Control', count: 3, tab: 'mppi' },
  { id: 'interaction', label: 'Manipulation & flight', count: 3, tab: 'arm' },
];
const ids = ['grid', 'sampling', 'parking', 'localization', 'slam', 'arena', 'mppi', 'admm', 'arm', 'pushing', 'drone'];

async function fits(page) {
  assert(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), 'showcase overflows horizontally');
  for (const button of await page.locator('[data-experiment-filter]').all()) {
    const box = await button.boundingBox();
    assert(box && box.height >= 44 && box.x >= 0 && box.x + box.width <= page.viewportSize().width,
      'topic button is too small or falls outside the screen');
  }
}

async function imageLoaded(page) {
  await page.waitForFunction(() => {
    const img = document.getElementById('feature-image');
    return img?.complete && img.naturalWidth > 0;
  });
}

export async function checkShowcase(page, home, shots, name) {
  await page.locator('[data-experiment-filter="all"][aria-pressed="true"]').waitFor();
  assert.deepEqual(await page.locator('.tile').evaluateAll((nodes) => nodes.map((node) => node.dataset.experiment)), ids);
  const beliefLink=new URL(await page.locator('.tile[data-experiment="localization"]').getAttribute('href'),home);
  assert.equal(beliefLink.searchParams.get('loc_view'),'belief');
  assert.equal(beliefLink.searchParams.get('belief_event'),'resample');
  await imageLoaded(page);
  await fits(page);
  if (shots) {
    await page.screenshot({ path: join(shots, `${name}-hero.png`) });
    await page.screenshot({ path: join(shots, `${name}-overview.png`), fullPage: true });
  }

  for (const topic of topics) {
    const button = page.locator(`[data-experiment-filter="${topic.id}"]`);
    // Native button keyboard activation must work as well as pointer/touch.
    if (topic.id === 'control') { await button.focus(); await page.keyboard.press('Space'); }
    else await button.click();
    assert.equal(await button.getAttribute('aria-pressed'), 'true');
    assert.equal(await page.locator('[data-experiment-filter][aria-pressed="true"]').count(), 1);
    assert.equal(new URL(page.url()).searchParams.get('topic'), topic.id);
    assert.equal(await page.locator('.tile').count(), topic.count);
    assert(await page.locator('.tile').evaluateAll((nodes, id) => nodes.every((node) => node.dataset.topic === id), topic.id));
    assert.equal(await page.locator('#featured-experiment').getAttribute('data-topic'), topic.id);
    await imageLoaded(page);
    await fits(page);

    const launch = page.locator('#feature-launch');
    const target = new URL(await launch.getAttribute('href'), home);
    assert.equal(target.origin, new URL(home).origin);
    assert.equal(target.pathname, '/rust_robotics/playground/');
    assert.equal(target.searchParams.get('tab'), topic.tab);
    if (shots) await page.locator('#featured-experiment').screenshot({ path: join(shots, `${name}-${topic.id}.png`) });
    await launch.click();
    await page.waitForURL(target.href);
    await page.waitForFunction(() => {
      const canvas = document.getElementById('rust_robotics_playground');
      return canvas?.width > 0 && canvas.height > 0 && !document.getElementById('loading');
    });
    // A real frame is painted after the loading placeholder has gone.
    await page.waitForTimeout(1000);
    if (topic.tab === 'sampling') {
      await page.waitForFunction(() => {
        const d=document.querySelector('canvas')?.dataset;
        return d?.rrtKind==='rrtstar' && d.rrtFrame==='249' && d.rrtPlaying==='0' && d.rrtSeed==='7';
      });
      const replay=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      assert.equal(JSON.parse(replay.rrtCosts).length,250);
      assert(JSON.parse(replay.rrtRewires).length>0,'the featured iteration did not show a real rewire');
    }
    if (topic.tab === 'arm') {
      await page.waitForFunction(() => document.querySelector('canvas')?.dataset.armCmpActive === '1');
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      assert.equal(d.armCmpPlaying,'0');assert.equal(d.armCmpFrame,'120');
      const s=JSON.parse(d.armCmpSamples);
      assert(s[0].error<0.1,'the featured joint tracker must be approaching the goal');
      assert.deepEqual(s[1].angles,[0,0],'the featured Cartesian run must show the actual radial stall');
      assert.equal(JSON.parse(d.armCmpErrors)[0].length,601);
    }
    if (topic.tab === 'slam') {
      await page.waitForFunction(() => document.querySelector('canvas')?.dataset.loopActive === '1');
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      assert.equal(d.loopPlaying,'0');assert.equal(d.loopVectors,'1');
      assert.equal(Number(d.loopFrame),JSON.parse(d.loopEvents)[0].frame);
      assert.equal(d.loopAccepted,'1');assert.equal(d.loopWrong,'0');assert.equal(d.loopOptimized,'1');
      assert.equal(d.loopPaintedFrame,d.loopFrame);
      const errors=JSON.parse(d.loopErrors),frame=Number(d.loopFrame);
      assert(errors[1][frame]<errors[0][frame]*0.5,'the featured loop must correct actual recorded error');
      assert.equal(JSON.parse(d.loopMaps).length,2);
    }
    if (topic.tab === 'admm') {
      await page.waitForFunction(() => document.querySelector('canvas')?.dataset.formationActive === '1');
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      assert.equal(d.formationFrame,'17');assert.equal(d.formationPlaying,'0');
      assert.deepEqual(JSON.parse(d.formationPainted),[17,0.25,true,true]);
      const runs=JSON.parse(d.formationRuns);
      assert.deepEqual(runs[0].plan.references,runs[1].plan.references);
      assert.equal(runs[0].history.length,18);
      assert.equal(JSON.parse(d.formationMaps).length,2);
    }
    if (topic.tab === 'parking') {
      await page.waitForFunction(() => document.querySelector('canvas')?.dataset.parkingActive === '1');
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset})),p=JSON.parse(d.parkingPlan);
      const boundary=p.changes.find((i)=>p.distance[i]>1e-9)??p.changes[0];
      assert.equal(Number(d.parkingCursor),boundary-1);
      assert.equal(d.parkingPlaying,'0');
      assert.deepEqual(JSON.parse(d.parkingPose),p.poses[boundary-1]);
      assert.deepEqual(JSON.parse(d.parkingPainted),[Number(d.parkingGeneration),boundary-1,true,false]);
      assert.equal(JSON.parse(d.parkingWitnesses).filter((w)=>w.kind==='body').length,4);
    }
    if (topic.tab === 'mppi') {
      await page.waitForFunction(() => document.querySelector('canvas')?.dataset.mppiActive === '1');
      const d=await page.locator('canvas').evaluate((c)=>({...c.dataset}));
      assert.equal(d.mppiCursor,'40');assert.equal(d.mppiHorizon,'40');
      const cloud=JSON.parse(d.mppiCloud);
      assert.equal(Number(d.mppiSample),cloud.order[0]);
      assert(Math.abs(cloud.weights.reduce((a,b)=>a+b,0)-1)<1e-9);
      assert.deepEqual(JSON.parse(d.mppiOrigin),JSON.parse(d.mppiRobot));
      assert.deepEqual(JSON.parse(d.mppiPainted),[Number(d.mppiGeneration),Number(d.mppiSample),40,true]);
    }
    if (shots) await page.screenshot({ path: join(shots, `${name}-${topic.id}-demo.png`) });
    await page.goBack({ waitUntil: 'load' });
    await page.locator(`[data-experiment-filter="${topic.id}"][aria-pressed="true"]`).waitFor();
    assert.equal(await page.locator('.tile').count(), topic.count, 'back navigation lost the topic');
  }

  await page.reload({ waitUntil: 'load' });
  await page.locator('[data-experiment-filter="interaction"][aria-pressed="true"]').waitFor();
  await page.locator('[data-experiment-filter="all"]').click();
  assert.equal(await page.locator('.tile').count(), 11);
  assert.equal(new URL(page.url()).searchParams.has('topic'), false);
  for (const link of await page.locator('.tile').all()) {
    const target = new URL(await link.getAttribute('href'), home);
    assert.equal(target.pathname, '/rust_robotics/playground/');
    assert(ids.includes(target.searchParams.get('tab')), 'tile points to an unknown experiment');
  }

  // Broad exploration still reaches the native implementations in the gallery.
  await page.locator('#category-filters').getByRole('button', { name: 'Control', exact: true }).click();
  const cards = page.locator('#gallery-grid .card');
  assert(await cards.count() > 0);
  assert(await cards.evaluateAll((nodes) => nodes.every((node) => node.querySelector('.card-category').textContent === 'Control')));
  await page.locator('#category-filters').getByRole('button', { name: 'All', exact: true }).click();
  await page.locator('#show-all').click();
  assert(await page.locator('#gallery-grid .card').count() > 12);

  // Both social previews reference a real artifact under the Pages prefix.
  const social = new URL(await page.locator('meta[property="og:image"]').getAttribute('content'));
  assert.equal(social.pathname, '/rust_robotics/assets/robotics-playground.png');
  assert.equal(await page.locator('meta[name="twitter:image"]').getAttribute('content'), social.href);
  const response = await page.request.get(new URL(social.pathname, home).href);
  assert.equal(response.status(), 200);
  assert.equal(response.headers()['content-type'], 'image/png');
  assert.deepEqual([...(await response.body()).subarray(0, 8)], [137, 80, 78, 71, 13, 10, 26, 10]);

  await page.goto(`${home}?topic=unknown`, { waitUntil: 'load' });
  await page.locator('[data-experiment-filter="all"][aria-pressed="true"]').waitFor();
  assert.equal(await page.locator('.tile').count(), 11);
  await page.emulateMedia({ reducedMotion: 'reduce' });
  assert.equal(await page.evaluate(() => getComputedStyle(document.documentElement).scrollBehavior), 'auto');
  await fits(page);
}

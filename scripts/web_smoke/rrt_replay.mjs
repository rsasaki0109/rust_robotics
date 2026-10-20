// Inspect real deterministic traces; all changes use the rendered canvas UI.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const canvas = '#rust_robotics_playground';
const state = (page) => page.locator(canvas).evaluate((c) => Object.fromEntries(
  Object.entries(c.dataset).filter(([key]) => key.startsWith('rrt'))));
const settle = (page) => page.waitForTimeout(100);
const receipts = new WeakMap();
const close = (actual, expected, message) => assert(Math.abs(actual - expected) < 8e-8, `${message}: ${actual} != ${expected}`);

function point(d, name, fraction) {
  if (fraction === undefined) return JSON.parse(d.rrtActions)[name];
  const [x, y, width, height] = JSON.parse(d.rrtPlot);
  return [x + width * fraction, y + height / 2];
}

async function aim(page, name, target, fraction) {
  // A layout revision can belong to hover, before a queued wheel is applied.
  // Process two real pointer moves, then require the actual widget hit test.
  // Never retry a tap: a delayed first tap could toggle a control twice.
  for (const dx of [-2, 0]) {
    const before = await state(page);
    // eframe reads MouseEvent.clientX/Y as integer CSS coordinates.
    const position = [Math.floor(target[0] + dx), Math.floor(target[1])];
    await page.mouse.move(...position);
    await page.waitForFunction(({ revision, position }) => {
      const d = document.querySelector('canvas')?.dataset;
      const pointer = JSON.parse(d?.rrtPointer ?? 'null');
      return Number(d?.rrtUiRevision) > revision && pointer && Math.hypot(pointer[0] - position[0], pointer[1] - position[1]) < 0.75;
    }, { revision: Number(before.rrtUiRevision), position });
    const after = await state(page);
    const current = point(after, name, fraction);
    if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) return false;
  }
  return JSON.parse((await state(page)).rrtHits)[name] === true;
}

async function scrollAction(page, name, fraction) {
  const before = await state(page);
  const previous = point(before, name, fraction)[1];
  // Leaving the drawing canvas can relayout between the scroll decision and
  // this processed hover. Recheck before asking a boundary to scroll further.
  if (previous >= 64 && previous <= page.viewportSize().height - 24) return;
  const delta = previous < 64 ? -280 : 280;
  await page.mouse.wheel(0, delta);
  await page.waitForFunction(({ revision, name, fraction, previous, delta }) => {
    const d = document.querySelector('canvas')?.dataset;
    if (!d || Number(d.rrtUiRevision) <= revision) return false;
    const y = fraction === undefined ? JSON.parse(d.rrtActions)[name][1]
      : (() => { const r = JSON.parse(d.rrtPlot); return r[1] + r[3] / 2; })();
    return delta < 0 ? y > previous + 1 : y < previous - 1;
  }, { revision: Number(before.rrtUiRevision), name, fraction, previous, delta });
}

async function action(page, name, touch, fraction) {
  for (let attempt = 0; attempt < 14; attempt++) {
    const target = point(await state(page), name, fraction);
    assert(target, `missing rrt action: ${name}`);
    const { width, height } = page.viewportSize();
    if (target[0] >= 0 && target[0] <= width && target[1] >= 64 && target[1] <= height - 24) {
      await settle(page);
      if (!await aim(page, name, target, fraction)) continue;
      const before = await state(page);
      const current = point(before, name, fraction);
      if (Math.hypot(current[0] - target[0], current[1] - target[1]) > 1) continue;
      if (!JSON.parse(before.rrtHits)[name]) continue;
      if (touch) await page.touchscreen.tap(...current);
      else await page.mouse.click(...current);
      await page.waitForFunction((count) => Number(document.querySelector('canvas')?.dataset.rrtActionCount) > count, Number(before.rrtActionCount));
      const after = await state(page);
      assert.equal(after.rrtLastAction, name, `input hit a different control than ${name}`);
      assert.equal(Number(after.rrtActionCount), Number(before.rrtActionCount) + 1, `${name} was handled more than once`);
      receipts.get(page)?.push({ name, count: Number(after.rrtActionCount), frame: Number(after.rrtFrame), playing: after.rrtPlaying,
        position: current, hit: JSON.parse(before.rrtHits)[name] });
      return after;
    }
    await page.mouse.move(Math.min(width - 20, Math.max(20, target[0])), height - 60);
    await scrollAction(page, name, fraction);
    await settle(page);
  }
  throw Error(`rrt action is unreachable: ${name}`);
}

const physical = (d) => Object.fromEntries(['rrtSeed','rrtFrame','rrtNodes','rrtPath','rrtCosts','rrtRewires','rrtRewireCount','rrtSample','rrtAccepted','rrtObstacles'].map((k) => [k,d[k]]));

function pointSegmentDistance(p,a,b) {
  const dx=b[0]-a[0],dy=b[1]-a[1],denom=dx*dx+dy*dy;
  const t=denom>0?Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom)):0;
  return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
}

function checkSearch(d) {
  assert.equal(d.rrtKind,'rrtstar');
  const frame=Number(d.rrtFrame),nodes=JSON.parse(d.rrtNodes),path=JSON.parse(d.rrtPath),costs=JSON.parse(d.rrtCosts);
  assert.equal(costs.length,frame+1,'the graph contains unobserved iterations');
  assert.equal(costs[0],null);
  assert(nodes.length>=1 && nodes.length<=frame+1);
  assert.equal(nodes[0][2],null);assert.equal(nodes[0][3],0);
  const obstacles=JSON.parse(d.rrtObstacles),radius=Number(d.rrtRobotRadius);
  const clear=(a,b)=>{for(const [x,y,r] of obstacles) assert(pointSegmentDistance([x,y],a,b)>=r+radius-1e-7,'a displayed edge intersects an inflated obstacle');};
  nodes.forEach((n,index)=>{
    assert(n.every((v)=>v===null || Number.isFinite(v)));
    if(index===0)return;
    assert(Number.isInteger(n[2]) && n[2]>=0 && n[2]<nodes.length && n[2]!==index);
    const parent=nodes[n[2]];
    close(n[3],parent[3]+Math.hypot(n[0]-parent[0],n[1]-parent[1]),'node cost follows its current parent');
    clear(parent,n);
    let cursor=index;const seen=new Set();
    while(nodes[cursor][2]!==null){assert(!seen.has(cursor),'parent cycle');seen.add(cursor);cursor=nodes[cursor][2];}
  });
  let best=Infinity,found=false;
  for(const cost of costs){
    if(cost===null){assert(!found,'a discovered path disappeared');continue;}
    assert(cost<=best+1e-7,'known path length increased');best=cost;found=true;
  }
  if(path.length){
    assert.deepEqual(path[0],JSON.parse(d.rrtGoal));assert.deepEqual(path.at(-1),JSON.parse(d.rrtStart));
    const cost=path.slice(1).reduce((sum,p,i)=>{clear(path[i],p);return sum+Math.hypot(p[0]-path[i][0],p[1]-path[i][1]);},0);
    close(cost,costs.at(-1),'displayed path length');
  }else assert.equal(costs.at(-1),null);
  for(const [index,before,after] of JSON.parse(d.rrtRewires)){assert.notEqual(before,after);assert.equal(nodes[index][2],after);}
}

async function sceneVisible(page) {
  for(let i=0;i<14;i++){
    const d=await state(page),r=JSON.parse(d.rrtRect);
    if(r[1]>=48 && r[1]+r[3]<=page.viewportSize().height-12){
      if(await aim(page,'field',JSON.parse(d.rrtActions).field)) return;
      continue;
    }
    await page.mouse.move(page.viewportSize().width/2,120);await page.mouse.wheel(0,-300);
    await page.waitForFunction(({revision,y})=>{
      const d=document.querySelector('canvas')?.dataset;
      return Number(d?.rrtUiRevision)>revision && JSON.parse(d.rrtRect)[1]>y+1;
    },{revision:Number(d.rrtUiRevision),y:r[1]});
  }
  throw Error('RRT* scene is unreachable');
}

async function dragGoal(page,touch){
  await sceneVisible(page);
  const before=await state(page),r=JSON.parse(before.rrtRect),g=JSON.parse(before.rrtGoal);
  const scale=r[2]/20;
  const start=[r[0]+g[0]*scale,r[1]+r[3]-g[1]*scale].map(Math.floor);
  // Convert the desired world target directly; rounding the start and then
  // subtracting a rounded displacement compounds a phone-sized pixel error.
  const end=[Math.round(r[0]+18.5*scale),Math.round(r[1]+r[3]-19.5*scale)];
  await page.mouse.move(...start);
  await page.waitForFunction(({revision,position})=>{
    const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.rrtPointer??'null');
    return Number(d?.rrtUiRevision)>revision && p && Math.hypot(p[0]-position[0],p[1]-position[1])<0.75;
  },{revision:Number(before.rrtUiRevision),position:start});
  const cdp=touch?await page.context().newCDPSession(page):null;
  try{
    if(cdp){await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});}
    else{await page.mouse.down();await page.mouse.move(...end,{steps:4});}
    await page.waitForFunction((goal)=>{
      const d=document.querySelector('canvas')?.dataset,g=JSON.parse(d?.rrtGoal??'null');
      return d?.rrtGoal!==goal && g && Math.abs(g[1]-19.5)<0.1;
    },before.rrtGoal);
    const held=await state(page);
    assert.equal(held.rrtPath,'[]','dragging the goal displayed a route for the previous goal');
    assert.equal(held.rrtNodes,'[]','an obsolete tree stayed visible while dragging');
    assert.equal(held.rrtEditing,'1');
  }finally{
    if(cdp){await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
    else await page.mouse.up();
  }
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.rrtActionCount)>count,Number(before.rrtActionCount));
  const after=await state(page),goal=JSON.parse(after.rrtGoal);
  assert.equal(after.rrtLastAction,'drag');assert.equal(Number(after.rrtActionCount),Number(before.rrtActionCount)+1);
  assert.equal(after.rrtEditing,'0');assert.equal(after.rrtFrame,'0');assert.equal(after.rrtPlaying,'0');
  assert(Math.abs(goal[0]-18.5)<0.1 && Math.abs(goal[1]-19.5)<0.1,'goal drag used wrong coordinates');
  checkSearch(after);
  receipts.get(page).push({name:'drag',count:Number(after.rrtActionCount),goal,touch});
  return after;
}

export async function checkRrtReplay(page,base,shots,touch=false,slow=false){
  receipts.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.rrtKind==='rrtstar');
  const initial=await state(page);assert.equal(initial.rrtFrame,'0');assert.equal(initial.rrtPlaying,'0');
  checkSearch(initial);assert.equal(JSON.parse(initial.rrtNodes).length,1);
  let d=await action(page,'step',touch);assert.equal(d.rrtFrame,'1');checkSearch(d);
  const first=await action(page,'first',touch);assert.equal(first.rrtFrame,first.rrtFirst);checkSearch(first);
  assert(JSON.parse(first.rrtCosts).slice(0,-1).every((v)=>v===null));
  const shorter=await action(page,'shorter',touch);checkSearch(shorter);
  assert(Number(shorter.rrtFrame)>Number(first.rrtFrame));assert(JSON.parse(shorter.rrtCosts).at(-1)<JSON.parse(first.rrtCosts).at(-1)-1e-7);
  const rewired=await action(page,'rewire',touch);assert(JSON.parse(rewired.rrtRewires).length>0);checkSearch(rewired);
  const back=await action(page,'back',touch);assert.equal(Number(back.rrtFrame),Number(rewired.rrtFrame)-1);checkSearch(back);
  d=await action(page,'cost-plot',touch,0.6);checkSearch(d);
  const clicked=receipts.get(page).at(-1).position,plot=JSON.parse(d.rrtPlot);
  assert.equal(Number(d.rrtFrame),Math.round((Math.floor(clicked[0])-plot[0])/plot[2]*700));
  const end=await action(page,'end',touch);assert.equal(end.rrtFrame,'700');assert.equal(end.rrtPlaying,'0');checkSearch(end);
  await action(page,'first',touch);await action(page,'slow',touch);
  const playing=await action(page,'play',touch);assert.equal(playing.rrtPlaying,'1');
  await page.waitForFunction((frame)=>Number(document.querySelector('canvas').dataset.rrtFrame)>frame+2,Number(playing.rrtFrame));
  const paused=await action(page,'play',touch);assert.equal(paused.rrtPlaying,'0');checkSearch(paused);
  await page.waitForTimeout(450);assert.deepEqual(physical(await state(page)),physical(paused),'pause did not freeze the replay');
  const nextSeed=await action(page,'new-seed',touch);assert.equal(nextSeed.rrtSeed,'8');assert.equal(nextSeed.rrtFrame,'0');assert.equal(nextSeed.rrtPlaying,'0');
  const seeded=await action(page,'end',touch);checkSearch(seeded);assert.notEqual(seeded.rrtNodes,end.rrtNodes,'new seed did not change exploration');
  await action(page,'reset',touch);
  const wall=await action(page,'wall',touch);assert.equal(wall.rrtFirst,'null');assert.equal(wall.rrtFrame,'0');
  const blocked=await action(page,'end',touch);checkSearch(blocked);assert.equal(blocked.rrtPath,'[]');assert(JSON.parse(blocked.rrtCosts).every((v)=>v===null));
  await action(page,'clear',touch);const clear=await action(page,'end',touch);checkSearch(clear);assert(JSON.parse(clear.rrtPath).length>=2);assert.equal(clear.rrtObstacles,'[]');
  await action(page,'reset',touch);const edited=await action(page,'field',touch);
  assert.equal(JSON.parse(edited.rrtObstacles).length,7,'the actual map tap did not remove an obstacle');assert.equal(edited.rrtFrame,'0');
  d=await action(page,'end',touch);checkSearch(d);
  await dragGoal(page,touch);
  const other=await action(page,'rrt',touch);assert.equal(other.rrtKind,'rrt');
  d=await action(page,'rrtstar',touch);assert.equal(d.rrtKind,'rrtstar');
  const reset=await action(page,'reset',touch);assert.equal(reset.rrtSeed,'7');assert.equal(reset.rrtFrame,'0');assert.equal(reset.rrtRate,'100');assert.equal(reset.rrtPlaying,'0');
  await action(page,'first',touch);await action(page,'shorter',touch);
  const saved=await action(page,'rewire',touch);checkSearch(saved);
  await sceneVisible(page);
  if(shots)await page.screenshot({path:join(shots,`rrt-replay${touch?'-phone':''}${slow?'-slow':''}-scene.png`)});
  const inputs=receipts.get(page);let paintTimes;
  if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3 && paintTimes.slice(1).every((v,i)=>v-paintTimes[i]>=350),'paint cap was not exercised');}
  await page.goto(`${base}?${saved.rrtShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.rrtKind==='rrtstar');
  const shared=await state(page);assert.deepEqual(physical(shared),physical(saved),'shared run is not the exact selected prefix');assert.equal(shared.rrtPlaying,'0');checkSearch(shared);
  if(shots)await writeFile(join(shots,`rrt-replay${touch?'-phone':''}${slow?'-slow':''}.json`),JSON.stringify({initial,first,shorter,rewired,blocked,reset,saved,shared,inputs,paintTimes},null,2));
}


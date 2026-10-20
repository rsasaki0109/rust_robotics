// Real mouse/touch experiments with independent two-link geometry checks.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state = (page) => page.locator('canvas').evaluate((c) => Object.fromEntries(
  Object.entries(c.dataset).filter(([key]) => key.startsWith('arm'))));
const inputs = new WeakMap();
const close = (a, b, label) => assert(Math.abs(a - b) < 1e-9, `${label}: ${a} != ${b}`);
const point = (d, name) => JSON.parse(d.armActions)[name];

function geometry(d) {
  const [a, b] = JSON.parse(d.armAngles), elbow = JSON.parse(d.armElbow);
  const tip = JSON.parse(d.armTip), target = JSON.parse(d.armTarget);
  close(elbow[0], Math.cos(a), 'elbow x'); close(elbow[1], Math.sin(a), 'elbow y');
  close(tip[0], Math.cos(a) + Math.cos(a + b), 'FK x');
  close(tip[1], Math.sin(a) + Math.sin(a + b), 'FK y');
  close(Math.hypot(tip[0] - elbow[0], tip[1] - elbow[1]), 1, 'second link length');
  close(Number(d.armError), Math.hypot(tip[0] - target[0], tip[1] - target[1]), 'tip error');
  close(Number(d.armDeterminant), Math.sin(b), 'Jacobian determinant');
  const reachable = Math.hypot(...target) <= 2;
  assert.equal(d.armReachable, reachable ? '1' : '0');
  for (const key of ['armSolution', 'armAlternate']) {
    const solution = JSON.parse(d[key]);
    if (!reachable) { assert.equal(solution, null, 'outside target fabricated an IK solution'); continue; }
    const [u, v] = solution;
    close(Math.cos(u) + Math.cos(u + v), target[0], `${key} x`);
    close(Math.sin(u) + Math.sin(u + v), target[1], `${key} y`);
  }
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.armPointer ?? 'null');
      return Number(d?.armUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.armUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).armHits)[name] === true;
}

async function visible(page, name) {
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing arm control ${name}`);
    if (p[0]>=0 && p[0]<=width && p[1]>=64 && p[1]<=height-24) {
      await page.waitForTimeout(100);
      if (await hover(page,name,p)) return point(await state(page),name);
      continue;
    }
    await page.mouse.move(Math.min(width-20,Math.max(20,p[0])),height-60);
    const latest=await state(page), y=point(latest,name)[1];
    if (y>=64 && y<=height-24) continue;
    const delta=y<64 ? -280 : 280;
    await page.mouse.wheel(0,delta);
    await page.waitForFunction(({revision,name,y,delta})=>{
      const d=document.querySelector('canvas')?.dataset;
      if (!d || Number(d.armUiRevision)<=revision) return false;
      const current=JSON.parse(d.armActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.armUiRevision),name,y,delta});
  }
  throw Error(`arm control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.armActionCount)>count,Number(before.armActionCount));
  const after=await state(page);
  assert.equal(after.armLastAction,name);
  assert.equal(Number(after.armActionCount),Number(before.armActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.armActionCount),position,hit:JSON.parse(before.armHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function sceneVisible(page) {
  for (let i=0;i<14;i++) {
    const d=await state(page), r=JSON.parse(d.armRect);
    if (r[1]>=48 && r[1]+r[3]<=page.viewportSize().height-12) return r;
    await page.mouse.move(page.viewportSize().width/2,120);
    await page.mouse.wheel(0,-300);
    await page.waitForFunction(({revision,y})=>{
      const d=document.querySelector('canvas')?.dataset;
      return Number(d?.armUiRevision)>revision && JSON.parse(d.armRect)[1]>y+1;
    },{revision:Number(d.armUiRevision),y:r[1]});
  }
  throw Error('arm scene unreachable');
}

async function drag(page, name, start, end, touch, predicate, argument) {
  assert(await hover(page,name,start,false),`${name} drag missed its actual widget`);
  const before=await state(page);
  if (touch) {
    const cdp=await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
    await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
    await page.waitForFunction(predicate,argument);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});
    await cdp.detach();
  } else {
    await page.mouse.move(...start); await page.mouse.down();
    await page.mouse.move(...end,{steps:4});
    await page.waitForFunction(predicate,argument);
    await page.mouse.up();
  }
  return acknowledged(page,name,before,end);
}

async function dragTarget(page,touch) {
  const r=await sceneVisible(page), before=await state(page), target=JSON.parse(before.armTarget);
  const screen=(p)=>[r[0]+r[2]/2+p[0]*r[2]/5.2,r[1]+r[3]/2-p[1]*r[2]/5.2].map(Math.round);
  // Convert the final world point directly, avoiding compounded pixel rounding.
  const end=screen([-0.75,1.25]);
  const d=await drag(page,'aim',screen(target),end,touch,()=>{
    const p=JSON.parse(document.querySelector('canvas').dataset.armTarget);
    return Math.hypot(p[0]+0.75,p[1]-1.25)<0.05;
  });
  assert.equal(d.armMode,'ik');
  return d;
}

async function dragJoint(page,name,touch) {
  const start=(await visible(page,name)).map(Math.round), before=await state(page);
  const index=name==='shoulder' ? 0 : 1, direction=index===0 ? -1 : 1;
  const end=[start[0]+(index===0 ? 30 : 40)*direction,start[1]];
  const d=await drag(page,name,start,end,touch,({index,direction,old})=>{
    const a=JSON.parse(document.querySelector('canvas').dataset.armAngles)[index];
    return a*direction>0.6 && Math.abs(a-old)>0.25;
  },{index,direction,old:JSON.parse(before.armAngles)[index]});
  assert.equal(d.armMode,'fk'); assert.equal(d.armPlaying,'0');
  assert(JSON.parse(d.armAngles)[index]*direction>0.6,'joint slider did not move');
  return d;
}

export async function checkArm(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.armActive==='1');
  const initial=await state(page); geometry(initial);
  assert.equal(initial.armPlaying,'0'); assert.equal(initial.armMode,'ik');
  assert.notDeepEqual(JSON.parse(initial.armSolution),JSON.parse(initial.armAlternate));
  if (shots) await page.screenshot({path:join(shots,`arm${touch?'-phone':''}${slow?'-slow':''}-initial.png`)});

  const step=await action(page,'step',touch);
  assert.equal(step.armSteps,'1'); close(Number(step.armElapsed),0.05,'step duration');
  assert(Number(step.armError)<Number(initial.armError));
  const positive=await action(page,'snap',touch);
  assert(Number(positive.armError)<1e-9);
  await action(page,'negative',touch);
  const negative=await action(page,'snap',touch);
  assert(JSON.parse(negative.armAngles)[1]<0);
  assert.notDeepEqual(JSON.parse(positive.armElbow),JSON.parse(negative.armElbow));
  JSON.parse(positive.armTip).forEach((v,i)=>close(v,JSON.parse(negative.armTip)[i],'branch-invariant target'));
  assert.equal((await action(page,'other',touch)).armOther,'0');
  assert.equal((await action(page,'other',touch)).armOther,'1');

  await action(page,'stretch',touch);
  const stretch=await action(page,'snap',touch);
  assert(Number(stretch.armError)<1e-9 && Math.abs(Number(stretch.armDeterminant))<1e-9);
  close(JSON.parse(stretch.armTip)[0],2,'full reach');
  await action(page,'fold',touch);
  const fold=await action(page,'snap',touch);
  assert(Number(fold.armError)<1e-9 && Math.abs(Number(fold.armDeterminant))<1e-9);
  const outside=await action(page,'outside',touch);
  assert.equal(outside.armReachable,'0'); assert.equal(outside.armPlaying,'0');
  const held=outside.armAngles;
  await page.waitForTimeout(900);
  assert.equal((await state(page)).armAngles,held,'outside target moved the arm');

  await action(page,'reset',touch);
  await action(page,'play',touch);
  await page.waitForFunction((error)=>{
    const d=document.querySelector('canvas').dataset;
    return Number(d.armSteps)>=3 && Number(d.armError)<error*0.8;
  },Number(initial.armError));
  const paused=await action(page,'play',touch);
  assert.equal(paused.armPlaying,'0');
  await page.waitForTimeout(900);
  const still=await state(page);
  assert.equal(still.armAngles,paused.armAngles,'paused pose moved');
  assert.equal(still.armElapsed,paused.armElapsed,'paused time advanced');
  assert.equal(still.armSteps,paused.armSteps,'paused simulation stepped');

  const aimed=await dragTarget(page,touch);
  assert.equal(aimed.armAngles,paused.armAngles,'paused target drag moved joints');
  const reached=await action(page,'snap',touch);
  assert(Number(reached.armError)<1e-9);
  await action(page,'fk',touch);
  const shoulder=await dragJoint(page,'shoulder',touch);
  const elbow=await dragJoint(page,'elbow',touch);
  assert.notEqual(elbow.armTip,shoulder.armTip);
  await action(page,'ik',touch);
  assert(Number((await action(page,'snap',touch)).armError)<1e-9);
  await action(page,'negative',touch);
  assert(Number((await action(page,'snap',touch)).armError)<1e-9);
  await action(page,'fk',touch);
  await dragJoint(page,'shoulder',touch);
  assert.equal((await action(page,'other',touch)).armOther,'0');
  const saved=await state(page);
  assert(Number(saved.armError)>0.1,'sharing must preserve a genuinely off-target manual pose');
  const reset=await action(page,'reset',touch);
  assert.equal(reset.armAngles,initial.armAngles); assert.equal(reset.armTarget,initial.armTarget);
  assert.equal(reset.armOther,'1'); assert.equal(reset.armMode,'ik');
  await sceneVisible(page);
  let paintTimes;
  if (slow) {
    paintTimes=await page.evaluate(()=>window.__slowPaintTimes);
    assert(paintTimes.length>=3 && paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow test did not cap paints');
  }
  await page.goto(`${base}?${saved.armShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.armActive==='1');
  const shared=await state(page); geometry(shared);
  for (const key of ['armAngles','armTarget','armBranch','armMode','armOther','armTip','armElbow','armSolution','armAlternate','armError','armDeterminant']) {
    assert.equal(shared[key],saved[key],`shared ${key} changed`);
  }
  assert.equal(shared.armPlaying,'0');
  assert.deepEqual(JSON.parse(shared.armTrail),[JSON.parse(shared.armTip)]);
  assert.equal(inputs.get(page).length,27);
  assert(inputs.get(page).every((i)=>i.hit),'a recorded input missed its widget');
  if (shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}`;
    await writeFile(join(shots,`arm${suffix}.json`),JSON.stringify({initial,positive,negative,stretch,fold,outside,paused,aimed,saved,shared,inputs:inputs.get(page),paintTimes},null,2));
  }
}

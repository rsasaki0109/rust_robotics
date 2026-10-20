// Recorded tracking runs checked through real mouse/touch input and independent math.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('armCmp'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.armCmpActions)[name];
const close=(a,b,label)=>assert(Math.abs(a-b)<1e-8,`${label}: ${a} != ${b}`);
const wrap=(v)=>{const r=(v+Math.PI)%(2*Math.PI);return (r<0?r+2*Math.PI:r)-Math.PI;};

function geometry(d) {
  if (d.armCmpActive==='1') {
    assert.equal(d.armCmpPaintedFrame,d.armCmpFrame,'drawn arms use a different time');
    assert.equal(d.armCmpPaintedGeneration,d.armCmpGeneration,'drawn arms use a different experiment');
  }
  const target=JSON.parse(d.armCmpTarget), samples=JSON.parse(d.armCmpSamples), curves=JSON.parse(d.armCmpErrors);
  const frame=Number(d.armCmpFrame), damping=Number(d.armCmpDamping), limit=Number(d.armCmpLimit);
  assert.equal(d.armCmpLast,'600'); assert.equal(d.armCmpDt,'0.01'); assert.equal(limit,2);
  assert.equal(curves.length,2);assert(curves.every((c)=>c.length===601 && c.every(Number.isFinite)));
  const reachable=Math.hypot(...target)<=2;
  for (const [i,s] of samples.entries()) {
    const [a,b]=s.angles, x=Math.cos(a)+Math.cos(a+b), y=Math.sin(a)+Math.sin(a+b);
    close(s.tip[0],x,'tip x');close(s.tip[1],y,'tip y');
    close(s.error,Math.hypot(x-target[0],y-target[1]),'error');
    close(s.error,curves[i][frame],'shared-time graph error');
    close(s.determinant,Math.sin(b),'Jacobian determinant');
    const j=[[-y,-Math.sin(a+b)],[x,Math.cos(a+b)]];
    for (const row of [0,1]) close(s.velocity[row],j[row][0]*s.rates[0]+j[row][1]*s.rates[1],'realized tip velocity');
    assert(Math.hypot(...s.rates)<=limit+1e-10,'joint-rate budget exceeded');
    if (!reachable || frame===600) {assert.deepEqual(s.rates,[0,0]);assert.deepEqual(s.requested,[0,0]);continue;}
    let q, requested;
    if (i===0) {
      const bGoal=(d.armCmpBranch==='positive'?1:-1)*Math.acos(Math.max(-1,Math.min(1,(target[0]**2+target[1]**2-2)/2)));
      const aGoal=target[0]===0 && target[1]===0 ? 0 : Math.atan2(target[1],target[0])-Math.atan2(Math.sin(bGoal),1+Math.cos(bGoal));
      q=[4*wrap(aGoal-a),4*wrap(bGoal-b)];
      requested=j.map((row)=>row[0]*q[0]+row[1]*q[1]);
    } else {
      requested=[4*(target[0]-x),4*(target[1]-y)];
      // Independent joint-space normal equations, rather than the Rust JJ^T solve.
      const m00=j[0][0]**2+j[1][0]**2+damping**2;
      const m01=j[0][0]*j[0][1]+j[1][0]*j[1][1];
      const m11=j[0][1]**2+j[1][1]**2+damping**2;
      const rhs=j[0].map((_,k)=>j[0][k]*requested[0]+j[1][k]*requested[1]);
      const det=m00*m11-m01*m01;
      q=[(m11*rhs[0]-m01*rhs[1])/det,(m00*rhs[1]-m01*rhs[0])/det];
    }
    requested.forEach((v,k)=>close(s.requested[k],v,'requested velocity'));
    const norm=Math.hypot(...q), scale=Math.min(1,limit/norm);
    q.forEach((v,k)=>close(s.rates[k],v*scale,'applied joint command'));
    assert.equal(s.limited,norm>limit);
  }
}

function nextStep(before,after) {
  const a=JSON.parse(before.armCmpSamples),b=JSON.parse(after.armCmpSamples);
  assert.equal(Number(after.armCmpFrame),Number(before.armCmpFrame)+1);
  a.forEach((s,i)=>s.angles.forEach((v,k)=>close(wrap(b[i].angles[k]-v),0.01*s.rates[k],'recorded command applied to next pose')));
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.armCmpPointer ?? 'null');
      return Number(d?.armCmpUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.armCmpUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).armCmpHits)[name] === true;
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
      if (!d || Number(d.armCmpUiRevision)<=revision) return false;
      const current=JSON.parse(d.armCmpActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.armCmpUiRevision),name,y,delta});
  }
  throw Error(`arm control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.armCmpActionCount)>count,Number(before.armCmpActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.armCmpActive==='0' || (d.armCmpPaintedFrame===d.armCmpFrame && d.armCmpPaintedGeneration===d.armCmpGeneration);
  });
  const after=await state(page);
  assert.equal(after.armCmpLastAction,name);
  assert.equal(Number(after.armCmpActionCount),Number(before.armCmpActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.armCmpActionCount),position,hit:JSON.parse(before.armCmpHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function sceneVisible(page) {
  for (let i=0;i<14;i++) {
    const d=await state(page), r=JSON.parse(d.armCmpRect);
    if (r[1]>=48 && r[1]+r[3]<=page.viewportSize().height-12) {
      // A wheel event can leave egui's smooth scroll moving after the scene
      // enters the viewport. Observe two processed pointer frames before
      // converting a world target into screen coordinates.
      if (!await hover(page,'aim',[r[0]+r[2]/2,r[1]+r[3]/2])) continue;
      const current=JSON.parse((await state(page)).armCmpRect);
      if (current.every((v,k)=>v===r[k])) return current;
      continue;
    }
    await page.mouse.move(page.viewportSize().width/2,120);
    await page.mouse.wheel(0,-300);
    await page.waitForFunction(({revision,y})=>{
      const d=document.querySelector('canvas')?.dataset;
      return Number(d?.armCmpUiRevision)>revision && JSON.parse(d.armCmpRect)[1]>y+1;
    },{revision:Number(d.armCmpUiRevision),y:r[1]});
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

async function graphSeek(page,fraction,touch) {
  for (let attempt=0;attempt<14;attempt++) {
    const d=await state(page),g=JSON.parse(d.armCmpGraph),height=page.viewportSize().height;
    const p=[Math.round(g[0]+fraction*g[2]),Math.round(g[1]+g[3]/2)];
    if (g[1]>=64 && g[1]+g[3]<=height-24) {
      if (!await hover(page,'graph',p,false)) continue;
      const before=await state(page);
      // Smooth scrolling may move the plot during the processed hover frames.
      // Recompute the target before any click, as sceneVisible does for drags.
      if (!JSON.parse(before.armCmpGraph).every((v,k)=>v===g[k])) continue;
      if (touch) await page.touchscreen.tap(...p);else await page.mouse.click(...p);
      const after=await acknowledged(page,'graph',before,p);
      assert(Math.abs(Number(after.armCmpFrame)-fraction*600)<=2,'graph selected a different time');
      assert.equal(after.armCmpPlaying,'0');return after;
    }
    const delta=g[1]<64 ? -260 : 260;
    await page.mouse.move(page.viewportSize().width/2,height-60);
    await page.mouse.wheel(0,delta);
    await page.waitForFunction(({revision,y,delta})=>{
      const d=document.querySelector('canvas')?.dataset;
      if (Number(d?.armCmpUiRevision)<=revision) return false;
      const next=JSON.parse(d.armCmpGraph)[1];return delta<0 ? next>y+1 : next<y-1;
    },{revision:Number(d.armCmpUiRevision),y:g[1],delta});
  }
  throw Error('comparison error graph unreachable');
}

async function dragTarget(page,touch) {
  const rect=await sceneVisible(page),before=await state(page),target=JSON.parse(before.armCmpTarget);
  const screen=(p)=>[rect[0]+rect[2]/2+p[0]*rect[2]/5.2,rect[1]+rect[3]/2-p[1]*rect[2]/5.2].map(Math.round);
  const d=await drag(page,'aim',screen(target),screen([-0.75,1.25]),touch,()=>{
    const p=JSON.parse(document.querySelector('canvas').dataset.armCmpTarget);
    return Math.hypot(p[0]+0.75,p[1]-1.25)<0.05;
  });
  assert.equal(d.armCmpFrame,'0');assert.equal(d.armCmpPlaying,'0');
  assert.equal(d.armCmpStart,before.armCmpStart,'goal drag changed the shared initial pose');
  return d;
}

export async function checkArmTracking(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.armCmpActive==='1');
  const initial=await state(page);geometry(initial);
  assert.equal(initial.armCmpFrame,'0');assert.equal(initial.armCmpPlaying,'0');
  const first=JSON.parse(initial.armCmpSamples);
  assert.deepEqual(first[0].angles,first[1].angles,'different initial conditions');
  const stepped=await action(page,'step',touch);nextStep(initial,stepped);
  const result=await action(page,'result',touch);
  assert.equal(result.armCmpFrame,'600');
  assert(JSON.parse(result.armCmpSamples).every((s)=>s.error<0.01),'reachable default run did not converge');
  await action(page,'start',touch);
  const low=await action(page,'low',touch),high=await action(page,'high',touch);
  assert.equal(low.armCmpDamping,'0.05');assert.equal(high.armCmpDamping,'0.6');
  const lowCurves=JSON.parse(low.armCmpErrors),highCurves=JSON.parse(high.armCmpErrors);
  assert.deepEqual(lowCurves[0],highCurves[0],'damping changed the joint controller');
  assert.notDeepEqual(lowCurves[1],highCurves[1],'damping had no Cartesian effect');
  const branch=await action(page,'negative',touch),branchCurves=JSON.parse(branch.armCmpErrors);
  assert.deepEqual(branchCurves[1],highCurves[1],'analytic branch changed the Cartesian controller');
  assert.notDeepEqual(branchCurves[0],highCurves[0]);
  await action(page,'positive',touch);
  const straight=await action(page,'straight',touch);
  const opening=await action(page,'step',touch);nextStep(straight,opening);
  assert.notDeepEqual(JSON.parse(opening.armCmpSamples)[0].angles,[0,0]);
  assert.deepEqual(JSON.parse(opening.armCmpSamples)[1].angles,[0,0]);
  const stalled=await action(page,'result',touch),stallSamples=JSON.parse(stalled.armCmpSamples);
  assert(stallSamples[0].error<0.01);close(stallSamples[1].error,0.8,'reachable radial stall');
  assert(JSON.parse(stalled.armCmpErrors)[1].every((v)=>Math.abs(v-0.8)<1e-10));
  const near=await action(page,'near',touch),escaped=await action(page,'result',touch);
  assert(JSON.parse(escaped.armCmpSamples)[1].error<JSON.parse(near.armCmpSamples)[1].error*0.5,'near-straight run did not escape');
  const outside=await action(page,'outside',touch),held=await action(page,'result',touch);
  for (const s of JSON.parse(held.armCmpSamples)) assert.deepEqual(s.angles,JSON.parse(outside.armCmpStart),'unreachable target moved joints');
  for (const curve of JSON.parse(held.armCmpErrors)) assert(curve.every((v)=>v===curve[0]));
  await action(page,'reach',touch);
  await action(page,'play',touch);
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.armCmpFrame)>=10);
  const paused=await action(page,'play',touch);
  assert.equal(paused.armCmpPlaying,'0');
  await page.waitForTimeout(900);
  const still=await state(page);
  assert.equal(still.armCmpFrame,paused.armCmpFrame,'paused clock advanced');
  assert.equal(still.armCmpSamples,paused.armCmpSamples,'paused commands or poses changed');
  const inspected=await graphSeek(page,0.3,touch);
  const next=await action(page,'step',touch);nextStep(inspected,next);
  const aimed=await dragTarget(page,touch);
  await action(page,'negative',touch);
  await graphSeek(page,0.24,touch);
  const saved=await state(page);
  if (shots) await page.screenshot({path:join(shots,`arm-tracking${touch?'-phone':''}${slow?'-slow':''}-scene.png`)});
  const live=await action(page,'live',touch);assert.equal(live.armCmpActive,'0');
  const liveState=await page.locator('canvas').evaluate((c)=>({angles:c.dataset.armAngles,target:c.dataset.armTarget,active:c.dataset.armActive}));
  assert.equal(liveState.active,'1');
  const captured=await action(page,'compare',touch);
  assert.equal(captured.armCmpActive,'1');assert.equal(captured.armCmpFrame,'0');
  assert.equal(captured.armCmpStart,liveState.angles);assert.equal(captured.armCmpTarget,liveState.target);
  const receipts=inputs.get(page);
  assert.equal(receipts.length,24);assert(receipts.every((i)=>i.hit));
  let paintTimes;
  if (slow) {
    paintTimes=await page.evaluate(()=>window.__slowPaintTimes);
    assert(paintTimes.length>=3 && paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');
  }
  await page.goto(`${base}?${saved.armCmpShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.armCmpActive==='1');
  const shared=await state(page);geometry(shared);
  for (const key of ['armCmpFrame','armCmpStart','armCmpTarget','armCmpBranch','armCmpDamping','armCmpSamples','armCmpErrors']) assert.equal(shared[key],saved[key],`shared ${key} changed`);
  assert.equal(shared.armCmpPlaying,'0');
  await page.goto(`${base}?tab=arm&arm_view=compare&arm_cmp_case=straight&arm_cmp_frame=120`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.armCmpActive==='1' && document.querySelector('canvas').dataset.armCmpFrame==='120');
  const featured=await state(page);geometry(featured);
  assert.deepEqual(JSON.parse(featured.armCmpSamples)[1].angles,[0,0]);
  assert(JSON.parse(featured.armCmpSamples)[0].error<0.1);
  if (shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}`;
    await page.screenshot({path:join(shots,`arm-tracking${suffix}-feature.png`)});
    await writeFile(join(shots,`arm-tracking${suffix}.json`),JSON.stringify({initial,result,low,high,branch,straight,stalled,near,escaped,outside,held,paused,aimed,saved,shared,featured,inputs:receipts,paintTimes},null,2));
  }
}


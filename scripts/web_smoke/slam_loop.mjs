// Inspect recorded SLAM through real input and independently check its geometry.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('loop'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.loopActions)[name];
const close=(a,b,label)=>assert(Math.abs(a-b)<1e-8,`${label}: ${a} != ${b}`);

function geometry(d) {
  if (d.loopActive==='1') {
    assert.equal(d.loopPaintedFrame,d.loopFrame,'maps use a different update');
    assert.equal(d.loopPaintedScenario,d.loopScenarioIndex,'maps use a different scenario');
    assert.equal(d.loopPaintedVectors,d.loopVectors,'correction arrows use a different setting');
  }
  const frame=Number(d.loopFrame),last=Number(d.loopLast),curves=JSON.parse(d.loopErrors);
  assert.equal(curves.length,2);
  assert(curves.every((c)=>c.length===last+1 && c.every(Number.isFinite)));
  const truth=JSON.parse(d.loopTruth),front=JSON.parse(d.loopFront),estimate=JSON.parse(d.loopEstimate);
  for (const [i,p] of [front,estimate].entries()) close(curves[i][frame],Math.hypot(p[0]-truth[0],p[1]-truth[1]),'position error');
  const nodes=JSON.parse(d.loopNodes),raw=JSON.parse(d.loopFrontNodes),nodeTruth=JSON.parse(d.loopNodeTruth),shifts=JSON.parse(d.loopShifts);
  assert(nodes.length>0);assert.equal(nodes.length,raw.length);assert.equal(nodes.length,nodeTruth.length);assert.equal(nodes.length,shifts.length);
  nodes.forEach((p,i)=>{assert(p.every(Number.isFinite));close(shifts[i],Math.hypot(p[0]-raw[i][0],p[1]-raw[i][1]),'node shift');});
  const events=JSON.parse(d.loopEvents),prefix=events.filter((e)=>e.frame<=frame);
  assert.equal(Number(d.loopAccepted),prefix.length,'map contains a future loop');
  for (const e of prefix) {
    assert(e.from<nodes.length && e.to<nodes.length,'edge references a future node');
    const a=nodeTruth[e.from],b=nodeTruth[e.to],dx=b[0]-a[0],dy=b[1]-a[1];
    const relative=[Math.cos(a[2])*dx+Math.sin(a[2])*dy,-Math.sin(a[2])*dx+Math.cos(a[2])*dy];
    assert.equal(e.wrong,Math.hypot(relative[0]-e.relative[0],relative[1]-e.relative[1])>0.5,'false loop was not judged from truth');
    assert(e.inlier>=0 && e.inlier<=1 && e.residual>=0);
  }
  assert.equal(Number(d.loopWrong),prefix.filter((e)=>e.wrong).length);
  assert.equal(d.loopOptimized,events.some((e)=>e.frame===frame && e.optimized)?'1':'0');
  const witnesses=JSON.parse(d.loopWitnesses);
  assert.equal(witnesses.length,2);
  for (const [view,points] of witnesses.entries()) {
    assert.equal(points.length,3);
    for (const [i,w] of points.entries()) {
      assert(w.node<nodes.length);
      assert.deepEqual(w.pose,(view===0?raw:nodes)[w.node],'map uses a future optimized pose');
      assert.equal(w.node,witnesses[1-view][i].node);
      assert.deepEqual(w.local,witnesses[1-view][i].local,'maps use different scans');
      const [x,y]=w.local,[px,py,a]=w.pose;
      close(w.world[0],px+Math.cos(a)*x-Math.sin(a)*y,'map point x');
      close(w.world[1],py+Math.sin(a)*x+Math.cos(a)*y,'map point y');
    }
  }
  const maps=JSON.parse(d.loopMaps);assert.equal(maps.length,2);
  close(maps[0][2],maps[1][2],'map world x scale');close(maps[0][3],maps[1][3],'map world y scale');
  close(maps[0][3]/maps[0][2],22/32,'map aspect ratio');
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.loopPointer ?? 'null');
      return Number(d?.loopUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.loopUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).loopHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.loopActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing loop control ${name}`);
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
      if (!d || Number(d.loopUiRevision)<=revision) return false;
      const current=JSON.parse(d.loopActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.loopUiRevision),name,y,delta});
  }
  throw Error(`loop control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.loopActionCount)>count,Number(before.loopActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.loopActive==='0' || (d.loopPaintedFrame===d.loopFrame && d.loopPaintedScenario===d.loopScenarioIndex && d.loopPaintedVectors===d.loopVectors);
  });
  const after=await state(page);
  assert.equal(after.loopLastAction,name);
  assert.equal(Number(after.loopActionCount),Number(before.loopActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.loopActionCount),position,hit:JSON.parse(before.loopHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}



async function graphInput(page,fraction,touch,drag=false) {
  await visible(page,'graph');
  const d=await state(page),g=JSON.parse(d.loopGraph);
  const pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)];
  const end=pos(fraction),start=drag?pos(0.25):end;
  assert(await hover(page,'graph',start,false),'graph input missed the widget');
  const before=await state(page),expected=Math.round((end[0]-g[0])/g[2]*Number(before.loopLast));
  const reached=(f)=>Number(document.querySelector('canvas')?.dataset.loopFrame)===f;
  if (drag && touch) {
    const cdp=await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
    await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
    await page.waitForFunction(reached,expected);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});
    await cdp.detach();
  } else if (drag) {
    await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});
    await page.waitForFunction(reached,expected);await page.mouse.up();
  } else if (touch) await page.touchscreen.tap(...end); else await page.mouse.click(...end);
  const after=await acknowledged(page,'graph',before,end);
  assert.equal(Number(after.loopFrame),expected,'graph selected a different update');
  assert.equal(after.loopPlaying,'0');return after;
}

export async function checkSlamLoop(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.loopActive==='1');
  const initial=await state(page);geometry(initial);
  assert.equal(initial.loopFrame,'0');assert.equal(initial.loopPlaying,'0');assert.equal(initial.loopAccepted,'0');
  const events=JSON.parse(initial.loopEvents),firstFrame=events[0].frame;
  assert(events.length>=2,'fixture needs successive accepted loops');
  const before=await action(page,'before',touch);
  assert.equal(Number(before.loopFrame),firstFrame-1);assert.equal(before.loopAccepted,'0');
  JSON.parse(before.loopShifts).forEach((v)=>close(v,0,'graph changed before any accepted loop'));
  const first=await action(page,'step',touch);
  assert.equal(Number(first.loopFrame),firstFrame);assert.equal(first.loopAccepted,'1');assert.equal(first.loopOptimized,'1');
  const errors=JSON.parse(first.loopErrors);
  assert(errors[1][firstFrame]<errors[0][firstFrame]*0.5,'first loop did not reduce the actual pose error');
  assert(Math.max(...JSON.parse(first.loopShifts))>0.1,'accepted loop did not move historical map nodes');
  const back=await action(page,'back',touch);assert.equal(back.loopNodes,before.loopNodes,'backward seek leaked later optimized nodes');
  const jumped=await action(page,'first',touch);assert.equal(jumped.loopNodes,first.loopNodes);
  const vectors=await action(page,'vectors',touch);assert.equal(vectors.loopVectors,'1');assert.equal(vectors.loopNodes,first.loopNodes);
  const next=await action(page,'next',touch);assert.equal(Number(next.loopFrame),events[1].frame);
  assert.equal((await action(page,'previous',touch)).loopFrame,first.loopFrame);
  const result=await action(page,'result',touch);assert.equal(result.loopFrame,result.loopLast);assert.equal(Number(result.loopAccepted),events.length);
  await action(page,'start',touch);await action(page,'play',touch);
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.loopFrame)>=10);
  const paused=await action(page,'play',touch);assert.equal(paused.loopPlaying,'0');
  await page.waitForTimeout(900);
  const still=await state(page);assert.equal(still.loopFrame,paused.loopFrame);assert.equal(still.loopNodes,paused.loopNodes,'paused map changed');
  await graphInput(page,0.37,touch);
  const scrubbed=await graphInput(page,0.90,touch,true);
  await action(page,'aliased_off',touch);
  const falseLoop=await action(page,'false',touch);assert(Number(falseLoop.loopWrong)>0);
  const off=await action(page,'result',touch);assert(Number(off.loopWrong)>0);
  await action(page,'aliased_on',touch);
  const on=await action(page,'result',touch);assert.equal(on.loopWrong,'0');
  const offCurves=JSON.parse(off.loopErrors),onCurves=JSON.parse(on.loopErrors);
  assert.deepEqual(offCurves[0],onCurves[0],'ambiguity check changed the independent front end');
  assert.notDeepEqual(offCurves[1],onCurves[1],'ambiguity check did not affect the graph');
  assert.equal(off.loopTruth,on.loopTruth,'paired experiments changed ground truth');
  const checkedBefore=await action(page,'before',touch);assert.equal(checkedBefore.loopAccepted,'0');
  const checkedFirst=await action(page,'first',touch);assert.equal(checkedFirst.loopWrong,'0');
  assert.equal((await action(page,'vectors',touch)).loopVectors,'0');
  await action(page,'corridor',touch);await action(page,'first',touch);
  const saved=await graphInput(page,0.92,touch,true);
  if (shots) await page.screenshot({path:join(shots,`slam-loop${touch?'-phone':''}${slow?'-slow':''}-scene.png`)});
  assert.equal((await action(page,'single',touch)).loopActive,'0');
  const compared=await action(page,'compare',touch);assert.equal(compared.loopActive,'1');assert.equal(compared.loopFrame,saved.loopFrame);assert.equal(compared.loopNodes,saved.loopNodes);
  const receipts=inputs.get(page);
  assert.equal(receipts.length,26);assert(receipts.every((i)=>i.hit));
  let paintTimes;
  if (slow) {
    paintTimes=await page.evaluate(()=>window.__slowPaintTimes);
    assert(paintTimes.length>=3 && paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');
  }
  await page.goto(`${base}?${saved.loopShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.loopActive==='1');
  const shared=await state(page);geometry(shared);
  for (const key of ['loopFrame','loopScenario','loopVectors','loopTruth','loopFront','loopEstimate','loopNodes','loopFrontNodes','loopErrors','loopEvents','loopShifts','loopAccepted','loopWrong','loopWitnesses']) assert.equal(shared[key],saved[key],`shared ${key} changed`);
  assert.equal(shared.loopPlaying,'0');
  await page.goto(`${base}?tab=slam&algorithm=loop&loop_view=compare&loop_event=first&playing=0&loop_vectors=1`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.loopActive==='1');
  const featured=await state(page);geometry(featured);
  assert.equal(Number(featured.loopFrame),JSON.parse(featured.loopEvents)[0].frame);assert.equal(featured.loopPlaying,'0');assert.equal(featured.loopOptimized,'1');assert.equal(featured.loopVectors,'1');
  if (shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}`;
    await page.screenshot({path:join(shots,`slam-loop${suffix}-feature.png`)});
    await writeFile(join(shots,`slam-loop${suffix}.json`),JSON.stringify({initial,before,first,vectors,next,result,paused,scrubbed,falseLoop,off,on,checkedBefore,checkedFirst,saved,shared,featured,inputs:receipts,paintTimes},null,2));
  }
}

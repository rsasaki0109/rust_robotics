// Inspect actual MPC plans and applied positions through real mouse/touch input.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('formation'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.formationActions)[name];
const close=(a,b,label,tol=1e-7)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const goal=(k)=>k<=18?[k*0.18,0]:[18*0.18,(k-18)*0.18];
const offsets=[[0.6,0],[0,0.6],[-0.6,0],[0,-0.6]];

function geometry(d) {
  if(d.formationActive!=='1') return;
  const frame=Number(d.formationFrame),noise=Number(d.formationNoise),runs=JSON.parse(d.formationRuns);
  assert.deepEqual(JSON.parse(d.formationPainted),[frame,noise,d.formationHorizon==='1',d.formationObservations==='1'],'map painted a different plan');
  assert.equal(runs.length,2);
  const maps=JSON.parse(d.formationMaps),witnesses=JSON.parse(d.formationWitnesses);
  assert.equal(maps.length,2);assert.equal(witnesses.length,2);
  close(maps[0][2],maps[1][2],'equal world x scale');close(maps[0][3],maps[1][3],'equal world y scale');
  for(const [index,run] of runs.entries()) {
    assert.equal(run.history.length,frame+1,'trail contains future positions');
    assert.deepEqual(run.history.at(-1),run.center);
    assert.equal(run.errors.length,35);
    const mean=run.history.reduce((sum,p,k)=>sum+Math.hypot(p[0]-goal(k)[0],p[1]-goal(k)[1]),0)/(frame+1);
    let sq=0;
    for(let i=2;i<run.history.length;i++) for(let axis=0;axis<2;axis++) sq+=(run.history[i][axis]-2*run.history[i-1][axis]+run.history[i-2][axis])**2;
    close(run.mean,mean,'applied prefix tracking');
    close(run.rms,frame>1?Math.sqrt(sq/(frame-1)):0,'applied prefix second difference');
    close(run.errors[frame],Math.hypot(run.center[0]-goal(frame)[0],run.center[1]-goal(frame)[1]),'current error graph sample');
    const p=run.plan;
    if(frame===34) assert.equal(p,null,'final state must have no next solve');
    else {
      assert.deepEqual(p.center[0],run.center,'horizon not anchored at current state');
      assert.equal(p.center.length,10);assert.equal(p.trajectories.length,4);assert.equal(p.references.length,4);
      assert.equal(p.primal.length,p.iterations);assert.equal(p.dual.length,p.iterations);
      assert(p.primal.every((v)=>Number.isFinite(v)&&v>=0));assert(p.dual.every((v)=>Number.isFinite(v)&&v>=0));
      assert(p.primal.at(-1)<1e-7 && p.dual.at(-1)<1e-7,'actual solver did not converge');
      let residual=0;
      for(let i=0;i<4;i++) for(let t=0;t<10;t++) for(let axis=0;axis<2;axis++) {
        residual+=(p.trajectories[i][t][axis]-p.center[t][axis]-offsets[i][axis])**2;
        const observation=goal(frame+t)[axis]+offsets[i][axis]+noise*(axis===0?Math.sin(2.1*i+0.7*(frame+t)):Math.cos(1.3*i+0.9*(frame+t)));
        close(p.references[i][t][axis],observation,'shared deterministic observation');
      }
      close(Math.sqrt(residual),p.primal.at(-1),'primal from actual trajectories',1e-10);
      if(index===0) for(let t=1;t<10;t++) for(let axis=0;axis<2;axis++) {
        const average=p.references.reduce((sum,ref,i)=>sum+ref[t][axis]-offsets[i][axis],0)/4;
        close(p.center[t][axis],average,'unsmoothed consensus mean');
      }
    }
    const [x,y,w,h]=maps[index];close(w,h,'square map');
    const expected={agent:4,center:1,goal:1,horizon:p&&d.formationHorizon==='1'?10:0,command:p&&d.formationHorizon==='1'?4:0,observation:p&&d.formationObservations==='1'?4:0};
    for(const [kind,count] of Object.entries(expected)) assert.equal(witnesses[index].filter((w)=>w.kind===kind).length,count,`${kind} drawn in the wrong layer`);
    for(const witness of witnesses[index]) {
      const i=witness.index;
      const world={agent:run.agents[i],center:run.center,goal:goal(frame),horizon:p?.center[i],command:p?.trajectories[i]?.[1],observation:p?.references[i]?.[1]}[witness.kind];
      assert.deepEqual(witness.world,world,'painted geometry differs from actual solver data');
      close(witness.pixel[0],x+(world[0]+1)/6.5*w,'painted x',0.0002);
      close(witness.pixel[1],y+h-(world[1]+1)/6.5*h,'painted y',0.0002);
    }
  }
  if(runs[0].plan) assert.deepEqual(runs[0].plan.references,runs[1].plan.references,'controllers saw different observations');
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.formationPointer ?? 'null');
      return Number(d?.formationUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.formationUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).formationHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.formationActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing formation control ${name}`);
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
      if (!d || Number(d.formationUiRevision)<=revision) return false;
      const current=JSON.parse(d.formationActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.formationUiRevision),name,y,delta});
  }
  throw Error(`formation control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.formationActionCount)>count,Number(before.formationActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.formationActive==='0' || JSON.stringify(JSON.parse(d.formationPainted ?? 'null'))===JSON.stringify([Number(d.formationFrame),Number(d.formationNoise),d.formationHorizon==='1',d.formationObservations==='1']);
  });
  const after=await state(page);
  assert.equal(after.formationLastAction,name);
  assert.equal(Number(after.formationActionCount),Number(before.formationActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.formationActionCount),position,hit:JSON.parse(before.formationHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}





async function graphInput(page,fraction,touch,drag=false) {
  await visible(page,'graph');
  const d=await state(page),g=JSON.parse(d.formationGraph);
  const pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)];
  const end=pos(fraction),start=drag?pos(0.2):end;
  assert(await hover(page,'graph',start,false),'graph input missed its widget');
  const before=await state(page),expected=Math.round((end[0]-g[0])/g[2]*34);
  const reached=(f)=>Number(document.querySelector('canvas')?.dataset.formationFrame)===f;
  if(drag&&touch) {
    const cdp=await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
    await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
    await page.waitForFunction(reached,expected);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
  } else if(drag) {
    await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});
    await page.waitForFunction(reached,expected);await page.mouse.up();
  } else if(touch) await page.touchscreen.tap(...end); else await page.mouse.click(...end);
  const after=await acknowledged(page,'graph',before,end);
  assert.equal(Number(after.formationFrame),expected);assert.equal(after.formationPlaying,'0');return after;
}

export async function checkFormationLab(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.formationActive==='1');
  const initial=await state(page);geometry(initial);assert.equal(initial.formationFrame,'0');assert.equal(initial.formationPlaying,'0');
  const before=await action(page,'before',touch);assert.equal(before.formationFrame,'17');
  const stepped=await action(page,'step',touch);assert.equal(stepped.formationFrame,'18');
  const plans=JSON.parse(before.formationRuns),next=JSON.parse(stepped.formationRuns);
  for(let i=0;i<2;i++) {
    assert.deepEqual(next[i].center,plans[i].plan.center[1],'step did not apply the displayed center command');
    assert.deepEqual(next[i].agents,plans[i].plan.trajectories.map((tr)=>tr[1]),'step did not apply displayed robot commands');
  }
  assert.equal((await action(page,'back',touch)).formationRuns,before.formationRuns,'backward seek leaked a future solve');
  await action(page,'corner',touch);
  assert.equal((await action(page,'horizon',touch)).formationHorizon,'0');
  assert.equal((await action(page,'observations',touch)).formationObservations,'0');
  await action(page,'horizon',touch);await action(page,'observations',touch);
  const heavy=await action(page,'heavy',touch);assert.equal(heavy.formationNoise,'0.5');
  const clean=await action(page,'clean',touch);assert.equal(clean.formationNoise,'0');
  assert.notEqual(clean.formationRuns,heavy.formationRuns,'changing noise did not change real plans');
  const noisy=await action(page,'noisy',touch);assert.equal(noisy.formationRuns,stepped.formationRuns);
  const result=await action(page,'result',touch);assert.equal(result.formationFrame,'34');
  const final=JSON.parse(result.formationRuns);assert(final[1].rms<final[0].rms,'smoothing did not reduce spatial second differences');
  await action(page,'start',touch);await action(page,'play',touch);
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.formationFrame)>=5);
  const paused=await action(page,'play',touch);assert.equal(paused.formationPlaying,'0');
  await page.waitForTimeout(900);assert.equal((await state(page)).formationRuns,paused.formationRuns,'paused plan changed');
  await graphInput(page,0.35,touch);const scrubbed=await graphInput(page,0.8,touch,true);
  const saved=await action(page,'before',touch);
  assert.equal((await action(page,'overlay',touch)).formationActive,'0');
  const inspected=await action(page,'inspect',touch);assert.equal(inspected.formationRuns,saved.formationRuns);
  // The final graph must be reachable below both maps, including a compact
  // desktop where the central scene and left controls scroll independently.
  await visible(page,'residual');
  assert.equal(JSON.parse((await state(page)).formationHits).residual,true);
  const receipts=inputs.get(page);assert.equal(receipts.length,20);assert(receipts.every((i)=>i.hit));
  let paintTimes;
  if(slow) {paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
  await page.goto(`${base}?${saved.formationShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.formationActive==='1');
  const shared=await state(page);geometry(shared);
  for(const key of ['formationFrame','formationNoise','formationHorizon','formationObservations','formationRuns']) assert.equal(shared[key],saved[key],`shared ${key} changed`);
  assert.equal(shared.formationPlaying,'0');
  if(shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
    await page.screenshot({path:join(shots,`formation-lab${suffix}-feature.png`)});
    await writeFile(join(shots,`formation-lab${suffix}.json`),JSON.stringify({initial,before,stepped,heavy,clean,noisy,result,paused,scrubbed,saved,shared,inputs:receipts,paintTimes},null,2));
  }
}

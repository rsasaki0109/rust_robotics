// Inspect a real frozen MPPI solve through mouse/touch, independently checking
// weights, dynamics, normalized controls and actual painter coordinates.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('mppi'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.mppiActions)[name];
const close=(a,b,label,tol=1e-8)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);

function geometry(d) {
  if(d.mppiActive!=='1')return;
  const c=JSON.parse(d.mppiCloud),sample=JSON.parse(d.mppiSelected),nominal=JSON.parse(d.mppiNominal),origin=JSON.parse(d.mppiOrigin);
  const id=Number(d.mppiSample),at=Number(d.mppiCursor),h=Number(d.mppiHorizon);
  assert.deepEqual(JSON.parse(d.mppiPainted),[Number(d.mppiGeneration),id,at,d.mppiShow==='1'],'map painted another decision');
  assert.deepEqual(JSON.parse(d.mppiRobot),origin,'inspection moved the robot');
  assert.equal(origin[2],0);assert.equal(origin[3],0);
  const n=c.costs.length;assert.equal(c.weights.length,n);assert.equal(c.first.length,n);assert.equal(c.order.length,n);
  const beta=Math.min(...c.costs),raw=c.costs.map((v)=>Math.exp(-(v-beta)/c.lambda)),z=raw.reduce((a,b)=>a+b,0);
  c.weights.forEach((w,i)=>{assert(Number.isFinite(w)&&w>=0);close(w,raw[i]/z,'normalized sample weight');});
  close(c.weights.reduce((a,b)=>a+b,0),1,'weight sum');
  close(c.ess,1/c.weights.reduce((sum,w)=>sum+w*w,0),'effective samples',1e-7);
  close(c.entropy,-c.weights.reduce((sum,w)=>sum+(w>0?w*Math.log(w):0),0),'weight entropy');
  assert.deepEqual(c.order,Array.from({length:n},(_,i)=>i).sort((a,b)=>c.costs[a]-c.costs[b]||a-b),'cost rank');
  assert.equal(sample.cost,c.costs[id]);assert.deepEqual(sample.controls[0],c.first[id]);
  close(sample.stage.reduce((a,b)=>a+b,0),sample.model,'stage cost total',1e-7);
  close(sample.model+sample.terminal,sample.cost,'reported cost decomposition');
  assert(sample.constraint>=0&&sample.model-sample.constraint>=-1e-7);
  for(const path of [sample,nominal]) {
    assert.equal(path.controls.length,h);assert.equal(path.states.length,h+1);assert.deepEqual(path.states[0],origin);
    path.controls.forEach((u,i)=>{
      assert(u.every((v)=>Number.isFinite(v)&&Math.abs(v)<=2+1e-12));
      const [x,y,vx,vy]=path.states[i];
      const expected=[x+vx*0.1+0.5*u[0]*0.01,y+vy*0.1+0.5*u[1]*0.01,vx+u[0]*0.1,vy+u[1]*0.1];
      expected.forEach((v,axis)=>close(path.states[i+1][axis],v,'double integrator forecast'));
    });
  }
  for(let axis=0;axis<2;axis++)close(nominal.controls[0][axis],c.first.reduce((sum,u,i)=>sum+c.weights[i]*u[axis],0),'weighted next command');
  const [x,y,w,height]=JSON.parse(d.mppiMap);close(w,height,'square map');
  const witnesses=JSON.parse(d.mppiWitnesses);assert.equal(witnesses.length,2*(h+1)+2);
  for(const witness of witnesses) {
    const selected=witness.kind.startsWith('selected'),path=selected?sample:nominal;
    assert.deepEqual(witness.state,path.states[witness.index],'painted path is not the reported prediction');
    if(witness.kind.endsWith('_cursor'))assert.equal(witness.index,at,'gold marker uses another forecast time');
    close(witness.pixel[0],x+w*witness.state[0]/20,'painted x',0.0002);
    close(witness.pixel[1],y+height-w*witness.state[1]/20,'painted y',0.0002);
  }
  assert.equal(Number(d.mppiDrawn),d.mppiShow==='1'?n:0,'cloud toggle did not change actual drawing');
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.mppiPointer ?? 'null');
      return Number(d?.mppiUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.mppiUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).mppiHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.mppiActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing mppi control ${name}`);
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
      if (!d || Number(d.mppiUiRevision)<=revision) return false;
      const current=JSON.parse(d.mppiActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.mppiUiRevision),name,y,delta});
  }
  throw Error(`mppi control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.mppiActionCount)>count,Number(before.mppiActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.mppiActive==='0' || JSON.stringify(JSON.parse(d.mppiPainted ?? 'null'))===JSON.stringify([Number(d.mppiGeneration),Number(d.mppiSample),Number(d.mppiCursor),d.mppiShow==='1']);
  });
  const after=await state(page);
  assert.equal(after.mppiLastAction,name);
  assert.equal(Number(after.mppiActionCount),Number(before.mppiActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.mppiActionCount),position,hit:JSON.parse(before.mppiHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}







async function chartInput(page,name,fraction,touch,drag=false) {
  await visible(page,name);
  const d=await state(page),g=JSON.parse(d.mppiCharts)[name==='time'?1:0];
  const pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)];
  const end=pos(fraction),start=drag?pos(0.2):end;
  assert(await hover(page,name,start,false),'chart input missed the widget');
  const before=await state(page),c=JSON.parse(before.mppiCloud);
  const index=Math.round((end[0]-g[0])/g[2]*(name==='time'?Number(before.mppiHorizon):c.order.length-1));
  const expected=name==='time'?index:c.order[index];
  const reached=({name,expected})=>Number(document.querySelector('canvas')?.dataset[name==='time'?'mppiCursor':'mppiSample'])===expected;
  if(drag&&touch) {
    const cdp=await page.context().newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
    await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
    await page.waitForFunction(reached,{name,expected});
    await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
  } else if(drag) {
    await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});
    await page.waitForFunction(reached,{name,expected});await page.mouse.up();
  } else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
  const after=await acknowledged(page,name,before,end);
  assert.equal(Number(after[name==='time'?'mppiCursor':'mppiSample']),expected);return after;
}

export async function checkMppiInspector(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mppiActive==='1');
  const initial=await state(page);geometry(initial);assert.equal(initial.mppiCursor,'0');
  const original=JSON.parse(initial.mppiCloud);
  const warm=await action(page,'warm',touch),cold=await action(page,'cold',touch),balanced=await action(page,'balanced',touch);
  const a=JSON.parse(cold.mppiCloud),b=JSON.parse(warm.mppiCloud);
  assert.deepEqual(a.costs,original.costs);assert.deepEqual(b.costs,original.costs);
  assert.deepEqual(a.first,original.first);assert.deepEqual(b.first,original.first);
  assert(a.ess<original.ess&&original.ess<b.ess,'temperature did not spread actual sample weight');
  assert.equal(balanced.mppiCloud,initial.mppiCloud);
  const best=await action(page,'best',touch);assert.equal(Number(best.mppiSample),original.order[0]);
  const worst=await action(page,'worst',touch);assert.equal(Number(worst.mppiSample),original.order.at(-1));
  assert.equal((await action(page,'nominal',touch)).mppiSample,'0');
  const ranked=await chartInput(page,'weights',0.4,touch);
  const forecast=await chartInput(page,'time',0.75,touch,true);
  assert.equal((await action(page,'cloud',touch)).mppiShow,'0');
  await action(page,'cloud',touch);
  const moving=await action(page,'moving',touch);assert.equal(moving.mppiMoving,'1');
  assert.deepEqual(JSON.parse(moving.mppiCloud).first,original.first,'predictions changed sampled controls');
  const staticAgain=await action(page,'moving',touch);assert.equal(staticAgain.mppiCloud,initial.mppiCloud);
  const reseeded=await action(page,'seed',touch);assert.equal(reseeded.mppiSeed,'19');
  assert.notDeepEqual(JSON.parse(reseeded.mppiCloud).first,original.first);
  const resetSeed=await action(page,'seed',touch);assert.equal(resetSeed.mppiCloud,initial.mppiCloud);
  assert.equal((await action(page,'start',touch)).mppiCursor,'0');
  const first=await action(page,'step',touch);assert.equal(first.mppiCursor,'1');
  assert.equal((await action(page,'back',touch)).mppiCursor,'0');
  const end=await action(page,'end',touch);assert.equal(end.mppiCursor,end.mppiHorizon);
  assert.equal((await action(page,'live',touch)).mppiActive,'0');
  const inspected=await action(page,'inspect',touch);assert.equal(inspected.mppiCloud,initial.mppiCloud);
  await action(page,'circle',touch);
  const saved=await chartInput(page,'weights',0.1,touch);
  await page.waitForTimeout(900);
  const still=await state(page);assert.equal(still.mppiCloud,saved.mppiCloud);assert.equal(still.mppiRobot,saved.mppiRobot);assert.equal(still.mppiNominal,saved.mppiNominal,'frozen decision advanced');
  await visible(page,'time');assert.equal(JSON.parse((await state(page)).mppiHits).time,true,'last chart unreachable');
  const receipts=inputs.get(page);assert.equal(receipts.length,22);assert(receipts.every((i)=>i.hit));
  let paintTimes;
  if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
  await page.goto(`${base}?${saved.mppiShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mppiActive==='1');
  const shared=await state(page);geometry(shared);
  for(const key of ['mppiSeed','mppiSample','mppiCursor','mppiHorizon','mppiShow','mppiMoving','mppiCloud','mppiOrigin','mppiRobot','mppiSelected','mppiNominal'])assert.equal(shared[key],saved[key],`shared ${key} changed`);
  await page.goto(`${base}?tab=mppi&mppi_inspect=1&mppi_case=circle&mppi_at=40`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mppiActive==='1');
  const featured=await state(page);geometry(featured);assert.equal(featured.mppiCursor,'40');
  assert.equal(Number(featured.mppiSample),JSON.parse(featured.mppiCloud).order[0]);
  if(shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
    await page.screenshot({path:join(shots,`mppi-inspector${suffix}-feature.png`)});
    await writeFile(join(shots,`mppi-inspector${suffix}.json`),JSON.stringify({initial,warm,cold,balanced,best,worst,ranked,forecast,moving,staticAgain,reseeded,resetSeed,first,end,inspected,saved,shared,featured,inputs:receipts,paintTimes},null,2));
  }
}

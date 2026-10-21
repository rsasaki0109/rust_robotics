// Real PF evidence, actual resampling ancestry and read-only painter witnesses.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('belief'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.beliefActions)[name];
const close=(a,b,label,tol=1e-8)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);

function geometry(d) {
  if(d.beliefActive!=='1')return;
  const phase=Number(d.beliefPhase),cursor=Number(d.beliefCursor),selected=Number(d.beliefSelected);
  const clouds=JSON.parse(d.beliefClouds),[prior,predicted,weighted,output]=clouds,p=clouds[phase],observations=JSON.parse(d.beliefObservations),sigma=Number(d.beliefSigma),parents=JSON.parse(d.beliefParents);
  assert.deepEqual(JSON.parse(d.beliefPainted),[Number(d.beliefGeneration),cursor,selected,d.beliefRanges==='1']);
  assert.equal(clouds.length,4);assert.equal(observations.length,Number(d.beliefLandmarks));
  assert.equal(Number(d.beliefLast),95);assert.equal(phase,cursor%4);
  for(const cloud of clouds){assert.equal(cloud.length,96);assert(cloud.every((p)=>p.length===5&&p.every(Number.isFinite)&&p[4]>=0));close(cloud.reduce((a,p)=>a+p[4],0),1,'weight normalization');}
  const raw=predicted.map((p)=>observations.reduce((w,[z,x,y])=>w*Math.exp(-0.5*((z-Math.hypot(p[0]-x,p[1]-y))/sigma)**2)/(sigma*Math.sqrt(2*Math.PI)),p[4]));
  const total=raw.reduce((a,b)=>a+b,0);assert(total>0,'Bayes weights underflowed');
  for(let i=0;i<96;i++){
    close(weighted[i][4],raw[i]/total,'Bayes posterior');assert.deepEqual(weighted[i].slice(0,4),predicted[i].slice(0,4));
    close(predicted[i][0],prior[i][0]+predicted[i][3]*Math.cos(prior[i][2])*0.25,'predicted x');
    close(predicted[i][1],prior[i][1]+predicted[i][3]*Math.sin(prior[i][2])*0.25,'predicted y');
  }
  const ess=1/weighted.reduce((s,p)=>s+p[4]**2,0);close(Number(d.beliefEss),ess,'effective sample size');
  assert.equal(parents!==null,ess<96*0.55,'wrong resampling threshold');
  if(parents){assert.equal(parents.length,96);assert(parents.every((a,i)=>Number.isInteger(a)&&a>=0&&a<96&&(!i||a>=parents[i-1])));output.forEach((p,i)=>{assert.deepEqual(p.slice(0,4),weighted[parents[i]].slice(0,4),'output differs from actual parent');close(p[4],1/96,'resampling uniform weight');});}
  else assert.deepEqual(output,weighted,'skipped resampling changed population');
  const [x,y,w,h]=JSON.parse(d.beliefMap);close(w,h,'square map',1e-5);
  const witnesses=JSON.parse(d.beliefWitnesses),particles=witnesses.filter((v)=>v.kind==='particle');assert.equal(particles.length,96);
  const ranges=witnesses.filter((v)=>v.kind==='range');assert.equal(ranges.length,phase>=2&&d.beliefRanges==='1'?observations.length:0);
  const mean=[0,1].map((k)=>p.reduce((s,p)=>s+p[k]*p[4],0));
  for(const v of witnesses){
    let expected;
    if(v.kind==='particle'){assert.deepEqual(v.state,p[v.index]);expected=p[v.index].slice(0,2);close(v.radius,Math.min(7,2+Math.sqrt(p[v.index][4]*96)),'particle radius',0.00002);}
    else if(v.kind==='range'){expected=v.world;const z=observations.find((o)=>o[1]===expected[0]&&o[2]===expected[1]);assert(z);close(v.range,z[0],'range sensor');close(v.radius,z[0]/13*w,'painted range radius',0.0002);}
    else if(v.kind==='estimate'){expected=mean;expected.forEach((a,k)=>close(v.world[k],a,'weighted mean'));}
    else if(v.kind==='truth')expected=JSON.parse(d.beliefTruth).slice(0,2);
    else assert.fail(`unexpected witness ${v.kind}`);
    close(v.pixel[0],x+(expected[0]+1)/13*w,'painted x',0.0002);close(v.pixel[1],y+h-(expected[1]+1)/13*h,'painted y',0.0002);
  }
  const [gx,gy,gw,gh]=JSON.parse(d.beliefGraph),max=Math.max(1/96,...p.map((p)=>p[4])),bars=JSON.parse(d.beliefBars);assert.equal(bars.length,96);
  bars.forEach(([i,weight,bx,by,bw,bh])=>{close(weight,p[i][4],'chart weight');close(bx,gx+i/96*gw,'bar x',0.0002);close(bw,gw/96,'bar width',0.0002);close(bh,p[i][4]/max*(gh-6),'bar height',0.0002);close(by,gy+gh-bh,'bar y',0.0002);});
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.beliefPointer ?? 'null');
      return Number(d?.beliefUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.beliefUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).beliefHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.beliefActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing belief control ${name}`);
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
      if (!d || Number(d.beliefUiRevision)<=revision) return false;
      const current=JSON.parse(d.beliefActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.beliefUiRevision),name,y,delta});
  }
  throw Error(`belief control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.beliefActionCount)>count,Number(before.beliefActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.beliefActive==='0' || JSON.stringify(JSON.parse(d.beliefPainted ?? 'null'))===JSON.stringify([Number(d.beliefGeneration),Number(d.beliefCursor),Number(d.beliefSelected),d.beliefRanges==='1']);
  });
  const after=await state(page);
  assert.equal(after.beliefLastAction,name);
  assert.equal(Number(after.beliefActionCount),Number(before.beliefActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.beliefActionCount),position,hit:JSON.parse(before.beliefHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}









async function graphInput(page,fraction,touch,drag=false) {
  for(let attempt=0;attempt<14;attempt++) {
    await visible(page,'graph');const d=await state(page),g=JSON.parse(d.beliefGraph);
    const pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)],start=pos(drag?0.15:fraction),end=pos(fraction);
    if(!await hover(page,'graph',start,false))continue;
    const before=await state(page);if(!JSON.parse(before.beliefGraph).every((v,k)=>v===g[k]))continue;
    const expected=Math.min(95,Math.floor((end[0]-g[0])/g[2]*96));
    const reached=(n)=>Number(document.querySelector('canvas')?.dataset.beliefSelected)===n;
    if(drag&&touch){
      const cdp=await page.context().newCDPSession(page);
      await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
      await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
      await page.waitForFunction(reached,expected);
      await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
    }else if(drag){await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});await page.waitForFunction(reached,expected);await page.mouse.up();}
    else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
    const after=await acknowledged(page,'graph',before,end);assert.equal(Number(after.beliefSelected),expected);assert.equal(after.beliefPlaying,'0');return after;
  }
  throw Error('belief chart did not settle before input');
}

export async function checkBeliefLab(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.beliefActive==='1');
  const initial=await state(page);geometry(initial);assert.equal(initial.beliefCursor,'0');assert.equal(initial.beliefPlaying,'0');
  const predicted=await action(page,'predict',touch);assert.equal(predicted.beliefCursor,'1');
  const weighted=await action(page,'weight',touch);assert.equal(weighted.beliefCursor,'2');
  const best=await action(page,'best',touch),cloud=JSON.parse(best.beliefClouds)[2];
  close(cloud[Number(best.beliefSelected)][4],Math.max(...cloud.map((p)=>p[4])),'highest weight selection');
  const resampled=await action(page,'resample',touch);assert.equal(resampled.beliefCursor,'3');assert.notEqual(resampled.beliefParents,'null');
  assert.equal(resampled.beliefClouds,initial.beliefClouds);assert.equal(resampled.beliefGeneration,initial.beliefGeneration,'phase selection reran the filter');
  assert.equal((await action(page,'back',touch)).beliefCursor,'2');
  const selected=await graphInput(page,0.2,touch),dragged=await graphInput(page,0.72,touch,true);
  const hidden=await action(page,'ranges',touch);assert.equal(hidden.beliefRanges,'0');await action(page,'ranges',touch);
  const one=await action(page,'one',touch);assert.equal(one.beliefLandmarks,'1');
  assert.equal(one.beliefTruth,weighted.beliefTruth);assert.deepEqual(JSON.parse(one.beliefClouds).slice(0,2),JSON.parse(weighted.beliefClouds).slice(0,2));
  assert.deepEqual(JSON.parse(one.beliefObservations)[0],JSON.parse(weighted.beliefObservations)[0]);
  assert.notDeepEqual(JSON.parse(one.beliefClouds)[2],JSON.parse(weighted.beliefClouds)[2]);
  const two=await action(page,'two',touch);assert.equal(two.beliefLandmarks,'2');
  const all=await action(page,'all',touch);assert.equal(all.beliefClouds,initial.beliefClouds);
  const noisy=await action(page,'noisy',touch);assert.equal(noisy.beliefSigma,'0.65');assert.notEqual(noisy.beliefClouds,initial.beliefClouds);
  const quiet=await action(page,'quiet',touch);assert.equal(quiet.beliefSigma,'0.15');
  const normal=await action(page,'normal',touch);assert.equal(normal.beliefClouds,initial.beliefClouds);
  const seeded=await action(page,'seed',touch);assert.equal(seeded.beliefSeed,'8');assert.notEqual(seeded.beliefClouds,initial.beliefClouds);assert.equal(seeded.beliefTruth,weighted.beliefTruth);
  const event=await action(page,'event',touch);assert.equal(event.beliefPhase,'2');assert.notEqual(event.beliefParents,'null');
  const next=await action(page,'next_scan',touch);assert.equal(Number(next.beliefCursor),Number(event.beliefCursor)+4);
  assert.deepEqual(JSON.parse(next.beliefClouds)[0],JSON.parse(event.beliefClouds)[3],'next prior is not the previous posterior');
  const previous=await action(page,'prev_scan',touch);assert.equal(previous.beliefClouds,event.beliefClouds);
  await action(page,'start',touch);await action(page,'play',touch);
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.beliefCursor)>=3);
  const paused=await action(page,'play',touch);assert.equal(paused.beliefPlaying,'0');
  await page.waitForTimeout(900);const still=await state(page);assert.equal(still.beliefCursor,paused.beliefCursor);assert.equal(still.beliefClouds,paused.beliefClouds);
  const particle=await action(page,'prev_particle',touch);assert.equal(Number(particle.beliefSelected),Number(paused.beliefSelected)-1);
  assert.equal((await action(page,'live',touch)).beliefActive,'0');
  const inspected=await action(page,'inspect',touch);assert.equal(inspected.beliefActive,'1');assert.equal(inspected.beliefSeed,'8');
  await action(page,'weight',touch);await action(page,'best',touch);
  const saved=await graphInput(page,0.4,touch);const receipts=inputs.get(page);assert.equal(receipts.length,28);assert(receipts.every((i)=>i.hit));
  let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350));}
  await page.goto(`${base}?${saved.beliefShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.beliefActive==='1');const shared=await state(page);geometry(shared);
  for(const key of ['beliefSeed','beliefSigma','beliefLandmarks','beliefCursor','beliefSelected','beliefRanges','beliefClouds','beliefTruth','beliefObservations','beliefParents','beliefEss','beliefTimeline'])assert.equal(shared[key],saved[key],`shared ${key} changed`);
  assert.equal(shared.beliefPlaying,'0');
  await page.goto(`${base}?tab=localization&loc_view=belief&belief_event=resample`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.beliefActive==='1');const featured=await state(page);geometry(featured);
  assert.equal(featured.beliefPhase,'2');assert.notEqual(featured.beliefParents,'null');assert.equal(featured.beliefClouds,initial.beliefClouds);
  if(shots){const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
    await page.screenshot({path:join(shots,`belief-lab${suffix}-feature.png`)});
    await writeFile(join(shots,`belief-lab${suffix}.json`),JSON.stringify({initial,predicted,weighted,best,resampled,selected,dragged,hidden,one,two,all,noisy,quiet,normal,seeded,event,next,previous,paused,particle,inspected,saved,shared,featured,inputs:receipts,paintTimes},null,2));}
}

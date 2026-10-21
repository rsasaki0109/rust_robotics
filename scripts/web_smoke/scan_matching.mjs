// Actual accepted ICP states, brute-force associations and painter geometry.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('icp'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.icpActions)[name];
const close=(a,b,label,tol=1e-8)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const apply=([x,y,a],[px,py])=>[Math.cos(a)*px-Math.sin(a)*py+x,Math.sin(a)*px+Math.cos(a)*py+y];
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);

function geometry(d) {
  if(d.icpActive!=='1')return;
  const source=JSON.parse(d.icpSource),target=JSON.parse(d.icpTarget),points=JSON.parse(d.icpPoints),pairs=JSON.parse(d.icpPairs),pose=JSON.parse(d.icpPose),initial=JSON.parse(d.icpInitial),truth=JSON.parse(d.icpTruth),errors=JSON.parse(d.icpErrors),cursor=Number(d.icpCursor),selected=Number(d.icpSelected);
  assert.deepEqual(JSON.parse(d.icpPainted),[Number(d.icpGeneration),cursor,selected,d.icpShowPairs==='1',d.icpGhost==='1']);
  assert.equal(points.length,source.length);assert.equal(pairs.length,source.length);assert.equal(errors.length,Number(d.icpLast)+1);
  assert(points.every((p)=>p.length===2&&p.every(Number.isFinite)));assert(pairs.every((j)=>Number.isInteger(j)&&j>=0&&j<target.length));
  const residuals=points.map((p,i)=>{
    apply(pose,source[i]).forEach((v,k)=>close(p[k],v,'rigid transform'));
    assert(target.some((q)=>distance(apply(truth,source[i]),q)<1e-8),'raw source differs from synthetic scan');
    const r=distance(p,target[pairs[i]]);close(r,Math.min(...target.map((q)=>distance(p,q))),'actual nearest neighbor');return r;
  });
  close(Number(d.icpMean),residuals.reduce((a,b)=>a+b,0)/source.length,'mean residual');close(errors[cursor],Number(d.icpMean),'recorded residual');
  for(let i=1;i<errors.length;i++)assert(errors[i]<=errors[i-1]+0.0001/source.length,'rejected fit was recorded');
  const [x,y,w,h]=JSON.parse(d.icpMap),screen=([a,b])=>[x+(a+7)/14*w,y+h-(b+7)/14*h];close(w,h,'square map',0.00002);
  const witnesses=JSON.parse(d.icpWitnesses);
  for(const [kind,n] of [['source',source.length],['target',target.length],['initial',d.icpGhost==='1'?source.length:0],['pair',d.icpShowPairs==='1'?source.length:1],['origin',d.icpGhost==='1'?3:2],['error',errors.length],['cursor',1]])assert.equal(witnesses.filter((v)=>v.kind===kind).length,n,`${kind} witness count`);
  const [gx,gy,gw,gh]=JSON.parse(d.icpGraph),emax=Math.max(0.05,...errors);
  for(const v of witnesses){
    if(v.kind==='error'||v.kind==='cursor'){
      const i=v.kind==='cursor'?cursor:v.index;assert.equal(v.index,i);close(v.pixel[0],gx+i/Math.max(1,errors.length-1)*gw,'curve x',0.0002);close(v.pixel[1],gy+gh-3-errors[i]/emax*(gh-6),'curve y',0.0002);continue;
    }
    if(v.kind==='pair'){
      if(d.icpShowPairs==='0')assert.equal(v.index,selected);
      assert.equal(v.target,pairs[v.index]);assert.deepEqual(v.world,[points[v.index],target[v.target]]);
      v.pixel.forEach((p,i)=>screen(v.world[i]).forEach((a,k)=>close(p[k],a,'pair pixel',0.0002)));continue;
    }
    let expected;
    if(v.kind==='source')expected=points[v.index];
    else if(v.kind==='target')expected=target[v.index];
    else if(v.kind==='initial')expected=apply(initial,source[v.index]);
    else if(v.kind==='origin'){
      const p=[initial,pose,truth][v.index];expected=p.slice(0,2);const end=apply(p,[0.65,0]);
      end.forEach((a,k)=>close(v.end[k],a,'pose heading'));screen(end).forEach((a,k)=>close(v.end_pixel[k],a,'heading pixel',0.0002));
    }else assert.fail(`unknown witness ${v.kind}`);
    expected.forEach((a,k)=>close(v.world[k],a,'world point'));screen(expected).forEach((a,k)=>close(v.pixel[k],a,'map pixel',0.0002));
  }
  const [rx,ry,rw,rh]=JSON.parse(d.icpResiduals),rmax=Math.max(0.05,...residuals),bars=JSON.parse(d.icpBars);assert.equal(bars.length,source.length);
  bars.forEach(([i,d,bx,by,bw,bh])=>{close(d,residuals[i],'point residual');close(bx,rx+i/source.length*rw,'bar x',0.0002);close(bw,rw/source.length,'bar width',0.0002);close(bh,d/rmax*(rh-4),'bar height',0.0002);close(by,ry+rh-bh,'bar y',0.0002);});
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.icpPointer ?? 'null');
      return Number(d?.icpUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.icpUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).icpHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.icpActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing icp control ${name}`);
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
      if (!d || Number(d.icpUiRevision)<=revision) return false;
      const current=JSON.parse(d.icpActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.icpUiRevision),name,y,delta});
  }
  throw Error(`icp control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.icpActionCount)>count,Number(before.icpActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.icpActive==='0' || JSON.stringify(JSON.parse(d.icpPainted ?? 'null'))===JSON.stringify([Number(d.icpGeneration),Number(d.icpCursor),Number(d.icpSelected),d.icpShowPairs==='1',d.icpGhost==='1']);
  });
  const after=await state(page);
  assert.equal(after.icpLastAction,name);
  assert.equal(Number(after.icpActionCount),Number(before.icpActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.icpActionCount),position,hit:JSON.parse(before.icpHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function chartInput(page,name,fraction,touch,drag=false) {
  for(let attempt=0;attempt<14;attempt++) {
    await visible(page,name);const d=await state(page),g=JSON.parse(d[name==='graph'?'icpGraph':'icpResiduals']);
    const pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)],start=pos(drag?0.15:fraction),end=pos(fraction);
    if(!await hover(page,name,start,false))continue;
    const before=await state(page);if(!JSON.parse(before[name==='graph'?'icpGraph':'icpResiduals']).every((v,k)=>v===g[k]))continue;
    const expected=name==='graph'?Math.round((end[0]-g[0])/g[2]*Number(before.icpLast)):Math.min(JSON.parse(before.icpSource).length-1,Math.floor((end[0]-g[0])/g[2]*JSON.parse(before.icpSource).length));
    const reached=({name,expected})=>Number(document.querySelector('canvas')?.dataset[name==='graph'?'icpCursor':'icpSelected'])===expected;
    if(drag&&touch){
      const cdp=await page.context().newCDPSession(page);
      await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
      await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
      await page.waitForFunction(reached,{name,expected});
      await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
    }else if(drag){await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});await page.waitForFunction(reached,{name,expected});await page.mouse.up();}
    else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
    const after=await acknowledged(page,name,before,end);assert.equal(Number(after[name==='graph'?'icpCursor':'icpSelected']),expected);assert.equal(after.icpPlaying,'0');return after;
  }
  throw Error(`ICP ${name} chart did not settle before input`);
}

async function editPose(page,touch,drag) {
  for(let attempt=0;attempt<14;attempt++) {
    await visible(page,'pose');
    const d=await state(page),r=JSON.parse(d.icpMap),screen=([x,y])=>[Math.round(r[0]+(x+7)/14*r[2]),Math.round(r[1]+r[3]-(y+7)/14*r[3])];
    const start=screen(drag?[0.6,-0.3]:[1.2,-0.8]),end=drag?screen([2,-0.6]):start;
    assert(start[1]>=64&&end[1]>=64&&start[1]<page.viewportSize().height-12&&end[1]<page.viewportSize().height-12,'pose gesture outside visible scene');
    if(!await hover(page,'pose',start,false))continue;
    const before=await state(page);if(!JSON.parse(before.icpMap).every((v,k)=>v===r[k]))continue;
    if(drag){
      const pressed=({start,revision})=>{const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.icpPress??'null');return Number(d?.icpUiRevision)>revision&&d.icpDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
      const preview=({end,revision})=>{const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.icpPointer??'null');return Number(d?.icpUiRevision)>revision&&d.icpDrag!=='null'&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75;};
      if(touch){
        const cdp=await page.context().newCDPSession(page);
        await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
        await page.waitForFunction(pressed,{start,revision:Number(before.icpUiRevision)});
        await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
        await page.waitForFunction(preview,{end,revision:Number(before.icpUiRevision)});
        await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
      }else{
        await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,revision:Number(before.icpUiRevision)});
        await page.mouse.move(...end,{steps:4});await page.waitForFunction(preview,{end,revision:Number(before.icpUiRevision)});await page.mouse.up();
      }
    }else if(touch)await page.touchscreen.tap(...start);else await page.mouse.click(...start);
    const after=await acknowledged(page,'pose',before,end),p=JSON.parse(after.icpInitial);
    close(p[0],(start[0]-r[0])/r[2]*14-7,'initial x',0.00002);close(p[1],(r[1]+r[3]-start[1])/r[3]*14-7,'initial y',0.00002);
    close(p[2],drag?Math.atan2(start[1]-end[1],end[0]-start[0]):JSON.parse(before.icpInitial)[2],'initial heading');
    assert.equal(after.icpSource,before.icpSource,'initial edit changed raw source');assert.equal(after.icpTarget,before.icpTarget,'initial edit changed raw target');
    assert.equal(after.icpCursor,'0');assert.equal(after.icpPlaying,'0');assert.equal(Number(after.icpGeneration),Number(before.icpGeneration)+1);return after;
  }
  throw Error('ICP scene did not settle before initial-pose input');
}

export async function checkScanMatching(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.icpActive==='1');
  const initial=await state(page);geometry(initial);assert.equal(initial.icpCursor,'0');assert.equal(initial.icpPlaying,'0');assert(Number(initial.icpLast)>=4);
  const stepped=await action(page,'step',touch);assert.equal(stepped.icpCursor,'1');
  const aligned=await action(page,'end',touch);assert.equal(aligned.icpCursor,aligned.icpLast);assert(Number(aligned.icpMean)<1e-8);close(distance(JSON.parse(aligned.icpPose),JSON.parse(aligned.icpTruth)),0,'corner final position');
  assert.equal(aligned.icpGeneration,initial.icpGeneration);assert.equal(aligned.icpErrors,initial.icpErrors);
  assert.equal(Number((await action(page,'back',touch)).icpCursor),Number(initial.icpLast)-1);await action(page,'start',touch);
  const worst=await action(page,'worst',touch),points=JSON.parse(worst.icpPoints),target=JSON.parse(worst.icpTarget),pairs=JSON.parse(worst.icpPairs),residuals=points.map((p,i)=>distance(p,target[pairs[i]]));
  close(residuals[Number(worst.icpSelected)],Math.max(...residuals),'worst selection');
  assert.equal(Number((await action(page,'prev_point',touch)).icpSelected),Math.max(0,Number(worst.icpSelected)-1));
  const ranked=await chartInput(page,'graph',0.5,touch),scrubbed=await chartInput(page,'graph',0.85,touch,true);
  const selected=await chartInput(page,'residuals',0.3,touch),pointDragged=await chartInput(page,'residuals',0.72,touch,true);
  assert.equal(pointDragged.icpGeneration,initial.icpGeneration);assert.equal(pointDragged.icpErrors,initial.icpErrors);
  assert.equal((await action(page,'pairs',touch)).icpShowPairs,'0');await action(page,'pairs',touch);
  assert.equal((await action(page,'ghost',touch)).icpGhost,'0');await action(page,'ghost',touch);
  const poor=await action(page,'poor',touch),stopped=await action(page,'end',touch);
  assert.equal(poor.icpSource,initial.icpSource);assert.equal(poor.icpTarget,initial.icpTarget);assert.deepEqual(JSON.parse(poor.icpInitial),[0,0,0]);
  assert.equal(stopped.icpConverged,'0');assert(Number(stopped.icpMean)>0.2);assert(distance(JSON.parse(stopped.icpPose),JSON.parse(stopped.icpTruth))>0.3);
  const partial=await action(page,'partial',touch),partialEnd=await action(page,'end',touch);assert(JSON.parse(partial.icpSource).length<JSON.parse(initial.icpSource).length);assert.equal(partial.icpTarget,initial.icpTarget);
  const bays=await action(page,'bays',touch),ambiguous=await action(page,'end',touch);assert.equal(ambiguous.icpConverged,'1');assert(Number(ambiguous.icpMean)<1e-8);close(distance(JSON.parse(ambiguous.icpPose),JSON.parse(ambiguous.icpTruth)),3,'repeated-bay position error');
  assert(JSON.parse(ambiguous.icpPairs).every((j)=>j>=JSON.parse(ambiguous.icpSource).length*2));
  await action(page,'corner',touch);const playing=await action(page,'play',touch);assert.equal(playing.icpPlaying,'1');
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.icpCursor)>=1);
  // Acknowledge a fresh hit at the Pause button without scrolling away while
  // this short, genuine trace plays. One processed pointer frame is enough.
  const pausePos=point(await state(page),'play'),rev=Number((await state(page)).icpUiRevision);await page.mouse.move(...pausePos);
  await page.waitForFunction((rev)=>{const d=document.querySelector('canvas')?.dataset;return Number(d?.icpUiRevision)>rev&&JSON.parse(d.icpHits).play;},rev);
  const pauseBefore=await state(page);if(touch)await page.touchscreen.tap(...pausePos);else await page.mouse.click(...pausePos);
  const paused=await acknowledged(page,'play',pauseBefore,pausePos);assert.equal(paused.icpPlaying,'0');assert(Number(paused.icpCursor)>0);
  await page.waitForTimeout(900);const still=await state(page);assert.equal(still.icpCursor,paused.icpCursor);assert.equal(still.icpPoints,paused.icpPoints);assert.equal(still.icpGeneration,paused.icpGeneration);
  const tapped=await editPose(page,touch,false),aimed=await editPose(page,touch,true),reset=await action(page,'reset_pose',touch);assert.equal(reset.icpInitial,initial.icpInitial);assert.equal(reset.icpErrors,initial.icpErrors);
  await action(page,'pairs',touch);await action(page,'ghost',touch);const picked=await chartInput(page,'residuals',0.45,touch);
  assert.equal((await action(page,'legacy',touch)).icpActive,'0');
  const inspected=await action(page,'inspect',touch);assert.equal(inspected.icpSource,initial.icpSource);assert.equal(inspected.icpShowPairs,'0');assert.equal(inspected.icpGhost,'0');
  const saved=await chartInput(page,'graph',0.5,touch),receipts=inputs.get(page);assert.equal(receipts.length,32);assert(receipts.every((i)=>i.hit));
  let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
  await page.goto(`${base}?${saved.icpShare}`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.icpActive==='1');
  const shared=await state(page);geometry(shared);for(const key of ['icpCase','icpInitial','icpSource','icpTarget','icpPoints','icpPairs','icpPose','icpErrors','icpCursor','icpSelected','icpShowPairs','icpGhost'])assert.equal(shared[key],saved[key],`shared ${key} changed`);assert.equal(shared.icpPlaying,'0');
  await page.goto(`${base}?tab=slam&algorithm=icp&icp_view=inspect`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.icpActive==='1');
  const featured=await state(page);geometry(featured);assert.equal(featured.icpInitial,initial.icpInitial);assert.equal(featured.icpCursor,'0');
  if(shots){const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;await page.screenshot({path:join(shots,`scan-matching${suffix}-feature.png`)});await writeFile(join(shots,`scan-matching${suffix}.json`),JSON.stringify({initial,stepped,aligned,worst,ranked,scrubbed,selected,pointDragged,poor,stopped,partial,partialEnd,bays,ambiguous,playing,paused,tapped,aimed,reset,picked,inspected,saved,shared,featured,inputs:receipts,paintTimes},null,2));}
}

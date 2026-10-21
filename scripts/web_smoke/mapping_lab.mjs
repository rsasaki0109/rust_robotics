// Independent sparse-ray union, float32 log-odds, and actual painted geometry.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const inputs=new WeakMap();
const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('mapping'))));
const point=(d,key)=>JSON.parse(d.mappingActions)[key];
const close=(a,b,label,tol=0.0003)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const equal=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const f32=Math.fround;
const prob=(v)=>1/(1+Math.exp(-f32(v)));
function geometry(d){
 if(d.mappingActive!=='1')return;
 const load=(k)=>JSON.parse(d['mapping'+k]),width=Number(d.mappingWidth),height=Number(d.mappingHeight),cursor=Number(d.mappingCursor),last=Number(d.mappingLast),sensor=load('Sensor'),target=load('Return'),selected=load('Selected'),index=selected[1]*width+selected[0],packets=load('Packets'),updates=load('Updates'),hit=f32(Number(d.mappingHit)),miss=f32(Number(d.mappingMiss));
 assert.equal(width,24);assert.equal(height,16);assert.equal(last,20);assert.equal(d.mappingResolution,'0.5');assert.deepEqual(load('Thresholds'),[-0.3,0.6]);assert.equal(d.mappingClamp,'5');
 const values=Array(width*height).fill(0),seen=values.slice(),history=[],snapshots=[];
 for(let t=0;t<=last;t++){
  let expected=t===0 || (d.mappingCase==='absent'&&t>4)?[]:d.mappingCase==='conflict'?[[...target],[target[0]+3,target[1]+0.2]]:[-1,0,1].map((dy)=>[target[0]+(d.mappingCase==='clear'&&t>4?3:0),target[1]+dy]);
  if(d.mappingDuplicates==='1')expected.push(...expected.map((p)=>[...p]));
  assert.deepEqual(packets[t],expected,'actual return packet');
  const marks=new Map();
  for(const p of expected){
   let x=Math.floor(sensor[0]/0.5),y=Math.floor(sensor[1]/0.5),tx=Math.floor(p[0]/0.5),ty=Math.floor(p[1]/0.5),dx=Math.abs(tx-x),dy=Math.abs(ty-y),sx=Math.sign(tx-x),sy=Math.sign(ty-y),error=dx-dy;
   for(;;){const i=y*width+x,mark=x===tx&&y===ty?2:1;marks.set(i,Math.max(marks.get(i)??0,mark));if(mark===2)break;const e=2*error;if(e>=-dy){error-=dy;x+=sx;}if(e<=dx){error+=dx;y+=sy;}}
  }
  const actual=new Map(updates[t].map((u)=>[u[1]*width+u[0],u]));
  assert.equal(actual.size,updates[t].length,'one application per cell');assert.equal(actual.size,marks.size);
  for(const [i,mark] of marks){
   const u=actual.get(i);assert(u,'missing actual cell update');assert.equal(u[2],mark===2);assert.equal(f32(u[3]),values[i],'before');
   values[i]=f32(Math.max(-5,Math.min(5,f32(values[i]+(mark===2?hit:miss)))));seen[i]++;
   assert.equal(f32(u[4]),values[i],'after');
  }
  history.push(values[index]);snapshots.push({values:[...values],seen:[...seen],states:values.map((v)=>v>f32(0.6)?2:v<f32(-0.3)?1:0)});
 }
 const current=snapshots[cursor];assert.deepEqual(load('Values').map(f32),current.values);assert.deepEqual(load('Seen'),current.seen);assert.deepEqual(load('States'),current.states);assert.deepEqual(load('History').map(f32),history);
 close(Number(d.mappingProbability),prob(current.values[index]),'cell probability',1e-12);
 const selectedUpdate=updates[cursor].find((u)=>u[0]===selected[0]&&u[1]===selected[1])??null;
 assert.deepEqual(load('CurrentUpdate'),selectedUpdate);
 let occupied=false,cleared=null;for(let t=0;t<=last;t++){occupied ||=snapshots[t].states[index]===2;if(occupied&&snapshots[t].states[index]===1){cleared=t;break;}}assert.equal(load('Cleared'),cleared);
 const stats=[0,0,0,0];current.values.forEach((v,i)=>stats[current.seen[i]===0?0:current.states[i]===1?2:current.states[i]===2?3:1]++);assert.deepEqual(load('Stats'),stats);assert.equal(stats.reduce((a,b)=>a+b),width*height);
 const [mx,my,mw,mh]=load('Map'),[gx,gy,gw,gh]=load('Graph'),side=mw/width,screen=(p)=>[mx+p[0]/12*mw,my+mh-p[1]/8*mh],project=(t,v)=>[gx+t/last*gw,gy+gh-(f32(v)+5)/10*gh];
 close(mh/height,side,'square cells');
 const witnesses=load('Witnesses');assert.equal(witnesses.filter((w)=>w.kind==='cell').length,width*height);assert.equal(witnesses.filter((w)=>w.kind==='update').length,updates[cursor].length);assert.equal(witnesses.filter((w)=>w.kind==='ray').length,d.mappingRays==='1'?packets[cursor].length:0);
 assert.equal(witnesses.filter((w)=>w.kind==='history').length,(d.mappingFuture==='1'?last:cursor)+1);assert.equal(witnesses.filter((w)=>w.kind==='threshold').length,2);
 assert.equal(witnesses.filter((w)=>w.kind==='selection').length,1);assert.equal(witnesses.filter((w)=>w.kind==='cursor').length,1);
 for(const w of witnesses){
  if(w.kind==='cell'){
   const [x,y]=w.cell,i=y*width+x,p=screen([x*0.5,(y+1)*0.5]);close(w.pixel[0],p[0],'cell x');close(w.pixel[1],p[1],'cell y');close(w.size,side,'cell edge');assert.equal(f32(w.value),current.values[i]);assert.equal(w.seen,current.seen[i]);assert.equal(w.state,current.states[i]);close(w.probability,prob(w.value),'painted probability',1e-12);
   const pOcc=prob(w.value),rgb=w.seen===0?[34,44,57]:[49,115,172].map((a,k)=>Math.round(a*(1-pOcc)+[207,90,89][k]*pOcc));assert.deepEqual(w.rgb,rgb);
  }else if(w.kind==='update'||w.kind==='selection'){
   const [x,y]=w.cell,p=screen([x*0.5,(y+1)*0.5]),inset=w.kind==='update'?1:0;
   p.forEach((v,k)=>close(w.pixel[k],v+inset,'outline'));close(w.size,side-2*inset,'outline size');
   if(w.kind==='selection')assert.deepEqual(w.cell,selected);else assert.equal(w.hit,updates[cursor].find((u)=>u[0]===x&&u[1]===y)[2]);
  }else if(w.kind==='sensor'||w.kind==='target'){
   assert.deepEqual(w.world,w.kind==='sensor'?sensor:target);screen(w.world).forEach((v,k)=>close(w.pixel[k],v,'pose paint'));
  }else if(w.kind==='ray'){
   assert.deepEqual(w.world,[sensor,packets[cursor][w.index]]);w.world.forEach((p,i)=>screen(p).forEach((v,k)=>close(w.pixel[i][k],v,'ray paint')));
  }else if(w.kind==='history'||w.kind==='cursor'){
   const t=w.at;assert(t<=last);if(w.kind==='history'){assert.equal(f32(w.value),history[t]);assert.equal(w.future,t>cursor);}else assert.equal(t,cursor);
   project(t,history[t]).forEach((v,k)=>close(w.pixel[k],v,'history paint'));
  }else if(w.kind==='threshold'){
   assert([-0.3,0.6].includes(w.value));const y=project(0,w.value)[1];[gx,y,gx+gw,y].forEach((v,k)=>close(w.pixel[k],v,'threshold paint'));
  }else assert.fail(`unexpected mapping witness ${w.kind}`);
 }
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.mappingPointer ?? 'null');
      return Number(d?.mappingUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.mappingUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).mappingHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.mappingActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing mapping control ${name}`);
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
      if (!d || Number(d.mappingUiRevision)<=revision) return false;
      const current=JSON.parse(d.mappingActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.mappingUiRevision),name,y,delta});
  }
  throw Error(`mapping control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.mappingActionCount)>count,Number(before.mappingActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.mappingActive==='0' || JSON.stringify(JSON.parse(d.mappingPainted ?? 'null'))===JSON.stringify([Number(d.mappingGeneration),Number(d.mappingCursor),JSON.parse(d.mappingSelected),d.mappingFuture==='1',d.mappingRays==='1']);
  });
  const after=await state(page);
  assert.equal(after.mappingLastAction,name);
  assert.equal(Number(after.mappingActionCount),Number(before.mappingActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.mappingActionCount),position,hit:JSON.parse(before.mappingHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function regionInput(page,key,from,to,touch,drag=false){
 const attribute='mapping'+key[0].toUpperCase()+key.slice(1);
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,key);const d=await state(page),r=JSON.parse(d[attribute]);
  const screen=(q)=>key==='map'?[Math.round(r[0]+q[0]/12*r[2]),Math.round(r[1]+r[3]-q[1]/8*r[3])]:[Math.round(r[0]+q*r[2]),Math.round(r[1]+r[3]/2)];
  const start=screen(from),end=screen(to),height=page.viewportSize().height;
  if(Math.min(start[1],end[1])<70||Math.max(start[1],end[1])>height-24){
   await page.mouse.move(Math.round(r[0]+r[2]/2),height-60);
   const before=await state(page),y=JSON.parse(before[attribute])[1],delta=Math.min(start[1],end[1])<70?-200:200;
   await page.mouse.wheel(0,delta);
   await page.waitForFunction(({attribute,y,revision})=>{const d=document.querySelector('canvas').dataset;return Number(d.mappingUiRevision)>revision&&Math.abs(JSON.parse(d[attribute])[1]-y)>1;},{attribute,y,revision:Number(before.mappingUiRevision)});
   continue;
  }
  if(!await hover(page,key,start,false))continue;
  const before=await state(page);if(!equal(JSON.parse(before[attribute]),r))continue;
  const expected=key==='map'?null:Math.min(Number(d.mappingLast),Math.round((end[0]-r[0])/r[2]*Number(d.mappingLast)));
  if(drag){
   const pressed=({start,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.mappingPress??'null');return Number(d.mappingUiRevision)>revision&&d.mappingDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
   const moved=({end,key,expected,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.mappingPointer??'null');return Number(d.mappingUiRevision)>revision&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75&&(key==='map'?d.mappingDrag!=='null':Number(d.mappingCursor)===expected);};
   if(touch){const cdp=await page.context().newCDPSession(page);await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await page.waitForFunction(pressed,{start,revision:Number(before.mappingUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.mappingUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
   else{await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,revision:Number(before.mappingUiRevision)});await page.mouse.move(...end,{steps:4});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.mappingUiRevision)});await page.mouse.up();}
  }else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
  const after=await acknowledged(page,key,before,end);
  if(key!=='map'){assert.equal(Number(after.mappingCursor),expected);assert.equal(after.mappingPlaying,'0');}
  return after;
 }throw Error(`mapping ${key} did not settle before input`);
}

export async function checkMappingLab(page,base,shots,touch=false,slow=false){
 inputs.set(page,[]);
 await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mappingActive==='1');
 const initial=await state(page);geometry(initial);assert.equal(initial.mappingCursor,'0');assert.equal(initial.mappingPlaying,'0');
 const first=await action(page,'first',touch);assert.equal(first.mappingCursor,'1');close(JSON.parse(first.mappingValues)[8*24+12],0.9,'first hit');
 const before=await action(page,'before',touch);assert.equal(before.mappingCursor,'4');
 const changed=await action(page,'change',touch);assert.equal(changed.mappingCursor,'5');assert.equal(JSON.parse(changed.mappingCurrentUpdate)[2],false);close(JSON.parse(changed.mappingHistory)[5],3.2,'old wall decreases');
 assert.equal(changed.mappingGeneration,initial.mappingGeneration);
 const result=await action(page,'end',touch);assert.equal(JSON.parse(result.mappingStates)[8*24+12],1);
 const sought=await regionInput(page,'graph',0.35,0.35,touch),scrubbed=await regionInput(page,'graph',0.15,0.5,touch,true);
 assert.equal((await action(page,'future',touch)).mappingFuture,'0');await action(page,'future',touch);
 const repeat=await action(page,'repeat',touch);close(JSON.parse(repeat.mappingHistory)[20],5,'hit saturation');
 const duplicated=await action(page,'duplicates',touch);assert.deepEqual(JSON.parse(duplicated.mappingUpdates),JSON.parse(repeat.mappingUpdates));assert(JSON.parse(duplicated.mappingPackets).every((p,t)=>p.length===JSON.parse(repeat.mappingPackets)[t].length*2));
 const unique=await action(page,'duplicates',touch);assert.equal(unique.mappingUpdates,repeat.mappingUpdates);
 const soft=await action(page,'soft_hit',touch);close(JSON.parse(soft.mappingHistory)[1],0.4,'soft hit');
 const strong=await action(page,'strong_hit',touch);close(JSON.parse(strong.mappingHistory)[1],1.4,'strong hit');
 const normal=await action(page,'normal_hit',touch);assert.equal(normal.mappingUpdates,repeat.mappingUpdates);
 await action(page,'gentle_clear',touch);await action(page,'fast_clear',touch);await action(page,'normal_clear',touch);
 const absent=await action(page,'absent',touch);const absentHistory=JSON.parse(absent.mappingHistory);assert(absentHistory.slice(4).every((v)=>v===absentHistory[4]));assert(JSON.parse(absent.mappingUpdates).slice(5).every((u)=>u.length===0));
 const conflict=await action(page,'conflict',touch);JSON.parse(conflict.mappingUpdates).slice(1).forEach((us)=>{const at=us.filter((u)=>u[0]===12&&u[1]===8);assert.equal(at.length,1);assert.equal(at[0][2],true);});
 await action(page,'clear',touch);const cleared=await action(page,'cleared',touch);assert.equal(cleared.mappingCursor,'14');assert.equal(JSON.parse(cleared.mappingStates)[8*24+12],1);
 await action(page,'inspect',touch);const inspected=await regionInput(page,'map',[5.25,4.25],[5.25,4.25],touch);assert.deepEqual(JSON.parse(inspected.mappingSelected),[10,8]);assert.equal(inspected.mappingGeneration,cleared.mappingGeneration);
 await action(page,'return',touch);const movedReturn=await regionInput(page,'map',[6.25,4.25],[6.5,4.5],touch,true);assert.equal(movedReturn.mappingCursor,'0');assert.equal(Number(movedReturn.mappingGeneration),Number(inspected.mappingGeneration)+1);
 await action(page,'sensor',touch);const movedSensor=await regionInput(page,'map',[2.25,3.25],[2.25,3.25],touch);assert.equal(Number(movedSensor.mappingGeneration),Number(movedReturn.mappingGeneration)+1);
 assert.notEqual(movedSensor.mappingSensor,initial.mappingSensor);assert.notEqual(movedReturn.mappingReturn,initial.mappingReturn);
 await action(page,'inspect',touch);const playing=await action(page,'play',touch);assert.equal(playing.mappingPlaying,'1');
 await page.waitForFunction(()=>Number(document.querySelector('canvas').dataset.mappingCursor)>=3);
 const saved=await action(page,'play',touch);assert.equal(saved.mappingPlaying,'0');assert(Number(saved.mappingCursor)>=3);
 assert.equal((await action(page,'legacy',touch)).mappingActive,'0');
 const reopened=await action(page,'inspect_lab',touch);assert.equal(reopened.mappingActive,'1');
 const keys=['mappingCase','mappingSensor','mappingReturn','mappingHit','mappingMiss','mappingDuplicates','mappingFuture','mappingRays','mappingEdit','mappingCursor','mappingSelected','mappingValues','mappingStates','mappingSeen','mappingHistory','mappingPackets','mappingUpdates','mappingStats','mappingCurrentUpdate'];
 for(const key of [...keys,'mappingGeneration'])assert.equal(reopened[key],saved[key],`returning changed ${key}`);
 const receipts=inputs.get(page);assert.equal(receipts.length,32);assert(receipts.every((r)=>r.hit));assert.deepEqual(receipts.map((r)=>r.count),Array.from({length:32},(_,i)=>i+1));
 let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
 await page.goto(`${base}?${saved.mappingShare}`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mappingActive==='1');const shared=await state(page);geometry(shared);
 for(const key of keys)assert.equal(shared[key],saved[key],`shared ${key} changed`);assert.equal(shared.mappingPlaying,'0');
 const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
 if(shots)await page.screenshot({path:join(shots,`mapping-lab${suffix}-shared.png`)});
 await page.route('https://fonts.googleapis.com/**',(route)=>route.fulfill({contentType:'text/css',body:''}));
 await page.goto(base.replace(/playground\/$/,''),{waitUntil:'load'});await page.locator('#play-occupancy').click();
 await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.mappingActive==='1');const featured=await state(page);geometry(featured);assert.equal(featured.mappingCase,'clear');assert.equal(featured.mappingCursor,'5');close(JSON.parse(featured.mappingHistory)[5],3.2,'featured fading wall');
 if(shots){await page.screenshot({path:join(shots,`mapping-lab${suffix}.png`)});await writeFile(join(shots,`mapping-lab${suffix}.json`),JSON.stringify({initial,first,before,changed,result,sought,scrubbed,repeat,duplicated,unique,soft,strong,normal,absent,conflict,cleared,inspected,movedReturn,movedSensor,playing,saved,reopened,shared,featured,inputs:receipts,paintTimes},null,2));}
}

// Actual library SIPP paths, independently checked against a space-time BFS.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const inputs=new WeakMap();
const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('time'))));
const point=(d,key)=>JSON.parse(d.timeActions)[key];
const close=(a,b,label,tol=0.0003)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const equal=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
function oracle(d,aware){
 const width=Number(d.timeWidth),height=Number(d.timeHeight),horizon=Number(d.timeHorizon),start=JSON.parse(d.timeStart),goal=JSON.parse(d.timeGoal),gate=JSON.parse(d.timeGate),walls=new Set(JSON.parse(d.timeWalls).map(String)),begin=Number(d.timeClose),end=begin+Number(d.timeDuration);
 const queue=[[...start,0]],seen=new Set([String(queue[0])]);
 for(let i=0;i<queue.length;i++){
  const [x,y,t]=queue[i];if(x===goal[0]&&y===goal[1])return t;if(t===horizon)continue;
  for(const [dx,dy] of [[0,0],[1,0],[-1,0],[0,1],[0,-1]]){
   const p=[x+dx,y+dy],nt=t+1,key=String([...p,nt]);
   if(p[0]<0||p[1]<0||p[0]>=width||p[1]>=height||walls.has(String(p))||(aware&&equal(p,gate)&&nt>=begin&&nt<end)||seen.has(key))continue;
   seen.add(key);queue.push([...p,nt]);
  }
 }return null;
}
function geometry(d){
 if(d.timeActive!=='1')return;
 const load=(k)=>JSON.parse(d['time'+k]),width=Number(d.timeWidth),height=Number(d.timeHeight),horizon=Number(d.timeHorizon),cursor=Number(d.timeCursor),begin=Number(d.timeClose),end=begin+Number(d.timeDuration),gate=load('Gate'),start=load('Start'),goal=load('Goal'),selected=load('Selected'),walls=new Set(load('Walls').map(String)),runs=load('Runs'),intervals=load('Intervals');
 assert.deepEqual(load('Painted'),[Number(d.timeGeneration),cursor,selected,d.timeFuture==='1']);
 assert.equal(width,13);assert.equal(height,7);assert.equal(horizon,24);assert.equal(Number(d.timeTickSeconds),0.5);assert.equal(intervals.length,width*height);
 const reserved=(p,t)=>walls.has(String(p))||(equal(p,gate)&&t>=begin&&t<end);
 for(let y=0;y<height;y++)for(let x=0;x<width;x++){
  const ivs=intervals[y*width+x];for(let i=0;i<ivs.length;i++){assert(ivs[i][0]<ivs[i][1]&&ivs[i][0]>=0&&ivs[i][1]<=horizon+1);if(i)assert(ivs[i-1][1]<ivs[i][0]);}
  for(let t=0;t<=horizon;t++)assert.equal(ivs.some(([a,b])=>a<=t&&t<b),!reserved([x,y],t),'safe interval complement');
 }
 assert.equal(runs.length,2);
 runs.forEach((r,run)=>{
  assert.equal(r.arrival,oracle(d,run===1),'SIPP earliest arrival differs from full space-time BFS');
  if(r.arrival===null){assert.equal(r.path.length,0);assert.equal(r.positions.length,0);assert.equal(r.position,null);assert(r.error);assert.equal(r.waiting,false);return;}
  assert.equal(r.error,null);assert.deepEqual(r.path[0],[...start,0]);assert.deepEqual(r.path.at(-1),[...goal,r.arrival]);assert.equal(r.positions.length,horizon+1);assert.equal(r.moves,r.path.length-1);assert.equal(r.moves+r.waits,r.arrival);
  let waypoint=0;
  for(let t=0;t<=horizon;t++){
   while(waypoint+1<r.path.length&&r.path[waypoint+1][2]<=t)waypoint++;
   assert.deepEqual(r.positions[t],r.path[waypoint].slice(0,2),'implicit wait reconstruction');assert(!walls.has(String(r.positions[t])));if(run===1)assert(!reserved(r.positions[t],t),'SIPP occupies reserved cell');
  }
  r.path.slice(1).forEach((p,i)=>{const a=r.path[i];assert.equal(Math.abs(p[0]-a[0])+Math.abs(p[1]-a[1]),1);assert(p[2]>a[2]);});
  assert.deepEqual(r.position,r.positions[cursor]);assert.deepEqual(r.conflicts,r.positions.flatMap((p,t)=>reserved(p,t)?[t]:[]));
  const observedEnd=Math.min(cursor,r.arrival),moves=r.positions.slice(1,observedEnd+1).filter((p,i)=>!equal(p,r.positions[i])).length;
  assert.equal(r.observed_moves,moves);assert.equal(r.observed_waits,observedEnd-moves);assert.equal(r.waiting,cursor<r.arrival&&equal(r.positions[cursor],r.positions[cursor+1]));
 });
 const first=runs[0].positions.findIndex((p,t)=>!equal(p,runs[1].positions[t]??null));assert.equal(Number(d.timeFirst),first<0?0:first);
 const [mx,my,mw,mh]=load('Map'),[bx,by,bw,bh]=load('Band'),[gx,gy,gw,gh]=load('Graph'),cell=mw/width,screen=(p)=>[mx+(p[0]+0.5)*cell,my+(p[1]+0.5)*cell];close(mh/height,cell,'square map cells');
 const phase=(r,t)=>r.arrival===null?'no plan':r.conflicts.includes(t)?'conflict':t>=r.arrival?'arrived':equal(r.positions[t],r.positions[t+1])?'wait':'move';
 const witnesses=load('Witnesses');assert.equal(witnesses.filter((w)=>w.kind==='cell').length,width*height);
 assert.equal(witnesses.filter((w)=>w.kind==='robot').length,runs.filter((r)=>r.arrival!==null).length);
 assert.equal(witnesses.filter((w)=>w.kind==='interval').length,intervals[selected[1]*width+selected[0]].length);
 const limit=d.timeFuture==='1'?horizon:cursor;
 for(let run=0;run<2;run++){
  assert.equal(witnesses.filter((w)=>w.kind==='path'&&w.run===run).length,runs[run].arrival===null?0:limit+1);
  assert.equal(witnesses.filter((w)=>w.kind==='phase'&&w.run===run).length,limit+1);
  const visits=runs[run].positions.slice(0,limit+1).filter((p)=>equal(p,selected)).length;assert.equal(witnesses.filter((w)=>w.kind==='visit'&&w.run===run).length,visits);
 }
 for(const w of witnesses){
  if(w.kind==='cell'){assert.equal(w.wall,walls.has(String(w.cell)));assert.equal(w.reserved,reserved(w.cell,cursor));const p=screen(w.cell);close(w.pixel[0],p[0]-cell/2,'cell x');close(w.pixel[1],p[1]-cell/2,'cell y');close(w.size,cell,'cell size');}
  else if(w.kind==='path'||w.kind==='robot'){assert.deepEqual(w.cell,w.kind==='robot'?runs[w.run].position:runs[w.run].positions[w.t]);screen(w.cell).forEach((v,k)=>close(w.pixel[k],v,'map paint'));if(w.kind==='robot'){assert.equal(w.conflict,runs[w.run].conflicts.includes(cursor));close(w.radius,cell*(w.run===0?0.24:0.36),'robot radius');}else assert.equal(w.future,w.t>cursor);}
  else if(w.kind==='interval'){assert(intervals[selected[1]*width+selected[0]].some((iv)=>equal(iv,w.interval)));const [a,b]=w.interval;[bx+a/(horizon+1)*bw,by,(b-a)/(horizon+1)*bw,bh].forEach((v,k)=>close(w.pixel[k],v,'interval paint'));}
  else if(w.kind==='visit'){assert.deepEqual(w.cell,selected);assert.deepEqual(runs[w.run].positions[w.t],selected);[bx+(w.t+0.5)/(horizon+1)*bw,by+bh*(w.run===0?0.3:0.7)].forEach((v,k)=>close(w.pixel[k],v,'visit paint'));}
  else if(w.kind==='phase'){assert.equal(w.status,phase(runs[w.run],w.t));[gx+w.t/(horizon+1)*gw,gy+w.run*36,gw/(horizon+1),24].forEach((v,k)=>close(w.pixel[k],v,'phase paint'));}
  else if(w.kind==='cursor'){const r=w.strip==='band'?[bx,by,bw,bh]:[gx,gy,gw,gh];const x=r[0]+(cursor+0.5)/(horizon+1)*r[2];assert.equal(w.t,cursor);[x,r[1],x,r[1]+r[3]].forEach((v,k)=>close(w.pixel[k],v,'cursor paint'));}
  else assert.fail(`unexpected time witness ${w.kind}`);
 }
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.timePointer ?? 'null');
      return Number(d?.timeUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.timeUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).timeHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.timeActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing time control ${name}`);
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
      if (!d || Number(d.timeUiRevision)<=revision) return false;
      const current=JSON.parse(d.timeActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.timeUiRevision),name,y,delta});
  }
  throw Error(`time control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.timeActionCount)>count,Number(before.timeActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.timeActive==='0' || JSON.stringify(JSON.parse(d.timePainted ?? 'null'))===JSON.stringify([Number(d.timeGeneration),Number(d.timeCursor),JSON.parse(d.timeSelected),d.timeFuture==='1']);
  });
  const after=await state(page);
  assert.equal(after.timeLastAction,name);
  assert.equal(Number(after.timeActionCount),Number(before.timeActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.timeActionCount),position,hit:JSON.parse(before.timeHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function regionInput(page,key,from,to,touch,drag=false){
 const attribute='time'+key[0].toUpperCase()+key.slice(1);
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,key);const d=await state(page),r=JSON.parse(d[attribute]);
  const screen=(q)=>key==='map'?[Math.round(r[0]+(q[0]+0.5)/Number(d.timeWidth)*r[2]),Math.round(r[1]+(q[1]+0.5)/Number(d.timeHeight)*r[3])]:[Math.round(r[0]+q*r[2]),Math.round(r[1]+r[3]/2)];
  const start=screen(from),end=screen(to),height=page.viewportSize().height;
  if(Math.min(start[1],end[1])<70||Math.max(start[1],end[1])>height-24){
   await page.mouse.move(Math.round(r[0]+r[2]/2),height-60);
   const before=await state(page),y=JSON.parse(before[attribute])[1],delta=Math.min(start[1],end[1])<70?-200:200;
   await page.mouse.wheel(0,delta);
   await page.waitForFunction(({attribute,y,revision})=>{const d=document.querySelector('canvas').dataset;return Number(d.timeUiRevision)>revision&&Math.abs(JSON.parse(d[attribute])[1]-y)>1;},{attribute,y,revision:Number(before.timeUiRevision)});
   continue;
  }
  if(!await hover(page,key,start,false))continue;
  const before=await state(page);if(!equal(JSON.parse(before[attribute]),r))continue;
  const expected=key==='map'?[Math.floor((end[0]-r[0])/r[2]*Number(d.timeWidth)),Math.floor((end[1]-r[1])/r[3]*Number(d.timeHeight))]:Math.min(Number(d.timeHorizon),Math.floor((end[0]-r[0])/r[2]*(Number(d.timeHorizon)+1)));
  if(drag){
   const pressed=({start,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.timePress??'null');return Number(d.timeUiRevision)>revision&&d.timeDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
   const moved=({end,key,expected,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.timePointer??'null');return Number(d.timeUiRevision)>revision&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75&&(key==='map'?d.timeDrag!=='null':Number(d.timeCursor)===expected);};
   if(touch){const cdp=await page.context().newCDPSession(page);await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await page.waitForFunction(pressed,{start,revision:Number(before.timeUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.timeUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
   else{await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,revision:Number(before.timeUiRevision)});await page.mouse.move(...end,{steps:4});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.timeUiRevision)});await page.mouse.up();}
  }else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
  const after=await acknowledged(page,key,before,end);
  if(key==='map')assert.deepEqual(JSON.parse(after.timeSelected),expected);
  else{assert.equal(Number(after.timeCursor),expected);assert.equal(after.timePlaying,'0');}
  return after;
 }throw Error(`time ${key} did not settle before input`);
}

export async function checkTimeLab(page,base,shots,touch=false,slow=false){
 inputs.set(page,[]);await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.timeActive==='1');
 const initial=await state(page);geometry(initial);assert.equal(initial.timeCursor,'0');assert.equal(initial.timePlaying,'0');
 const first=await action(page,'first',touch),f=JSON.parse(first.timeRuns);assert.equal(first.timeCursor,'5');assert.equal(f[0].conflicts[0],5);assert(f[1].waiting);assert.equal(f[1].observed_waits,1);
 const before=await action(page,'before',touch);assert.equal(before.timeCursor,'4');assert.deepEqual(...JSON.parse(before.timeRuns).map((r)=>r.position));
 const stepped=await action(page,'step',touch);assert.equal(stepped.timeCursor,'5');assert.equal(stepped.timeGeneration,initial.timeGeneration);
 const result=await action(page,'end',touch);assert(JSON.parse(result.timeRuns).every((r)=>equal(r.position,JSON.parse(result.timeGoal))));
 const sought=await regionInput(page,'graph',0.44,0.44,touch),scrubbed=await regionInput(page,'graph',0.18,0.26,touch,true);
 assert.equal((await action(page,'future',touch)).timeFuture,'0');await action(page,'future',touch);
 const long=await action(page,'long',touch),longRun=JSON.parse(long.timeRuns)[1];assert.equal(longRun.arrival,14);assert.equal(longRun.waits,0);assert(longRun.path.some((p)=>p[1]===1));
 const shorter=await action(page,'shorter',touch);assert.equal(shorter.timeDuration,'7');await action(page,'longer',touch);
 await action(page,'earlier',touch);await action(page,'later',touch);const timing=await action(page,'normal_schedule',touch);assert.deepEqual(JSON.parse(timing.timeRuns).map((r)=>r.path),JSON.parse(long.timeRuns).map((r)=>r.path));
 const corridor=await action(page,'corridor',touch);assert.equal(JSON.parse(corridor.timeRuns)[1].waits,3);
 const delayed=await action(page,'longer',touch);assert.equal(JSON.parse(delayed.timeRuns)[1].waits,4);assert.deepEqual(JSON.parse(delayed.timeRuns)[0].path,JSON.parse(corridor.timeRuns)[0].path);
 const closed=await action(page,'closed',touch),noPlan=JSON.parse(closed.timeRuns)[1];assert.equal(noPlan.arrival,null);assert.deepEqual(noPlan.positions,[]);assert(!JSON.parse(closed.timeWitnesses).some((w)=>w.kind==='robot'&&w.run===1));
 await action(page,'brief',touch);await action(page,'select',touch);const selected=await regionInput(page,'map',[5,3],[5,3],touch);assert.deepEqual(JSON.parse(selected.timeIntervals)[3*13+5],[[0,25]]);
 const band=await regionInput(page,'band',5.5/25,5.5/25,touch);assert.equal(band.timeCursor,'5');
 await action(page,'walls',touch);const tapped=await regionInput(page,'map',[6,1],[6,1],touch);assert(JSON.parse(tapped.timeWalls).some((p)=>equal(p,[6,1])));assert.equal(Number(tapped.timeGeneration),Number(band.timeGeneration)+1);
 await action(page,'long',touch);const painted=await regionInput(page,'map',[6,1],[6,2],touch,true),detoured=JSON.parse(painted.timeRuns)[1];assert.equal(detoured.arrival,18);assert.equal(detoured.waits,8);assert(painted.timeWalls!==long.timeWalls);
 await action(page,'select',touch);const inspectedCell=await regionInput(page,'map',[5,3],[5,3],touch);assert.equal(inspectedCell.timeGeneration,painted.timeGeneration);
 await action(page,'first',touch);const playing=await action(page,'play',touch);assert.equal(playing.timePlaying,'1');
 await page.waitForFunction(()=>Number(document.querySelector('canvas').dataset.timeCursor)>=8);
 const saved=await action(page,'play',touch);assert.equal(saved.timePlaying,'0');assert(Number(saved.timeCursor)>=8);assert.equal(saved.timeGeneration,painted.timeGeneration);
 assert.equal((await action(page,'legacy',touch)).timeActive,'0');const reopened=await action(page,'inspect',touch);assert.equal(reopened.timeActive,'1');
 for(const key of ['timeCase','timeClose','timeDuration','timeWalls','timeIntervals','timeRuns','timeCursor','timeSelected','timeFuture','timeEdit','timeGeneration'])assert.equal(reopened[key],saved[key],`returning to lab changed ${key}`);
 const receipts=inputs.get(page);assert.equal(receipts.length,32);assert(receipts.every((r)=>r.hit));assert.deepEqual(receipts.map((r)=>r.count),Array.from({length:32},(_,i)=>i+1));
 let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
 await page.goto(`${base}?${saved.timeShare}`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.timeActive==='1');const shared=await state(page);geometry(shared);
 for(const key of ['timeCase','timeClose','timeDuration','timeWalls','timeIntervals','timeRuns','timeCursor','timeSelected','timeFuture','timeEdit'])assert.equal(shared[key],saved[key],`shared ${key} changed`);assert.equal(shared.timePlaying,'0');
 const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
 if(shots)await page.screenshot({path:join(shots,`time-lab${suffix}-shared.png`)});
 await page.route('https://fonts.googleapis.com/**',(route)=>route.fulfill({contentType:'text/css',body:''}));await page.goto(base.replace(/playground\/$/,''),{waitUntil:'load'});await page.locator('#play-time-aware').click();
 await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.timeActive==='1');const featured=await state(page);geometry(featured);assert.equal(featured.timeCursor,featured.timeFirst);assert(JSON.parse(featured.timeRuns)[1].waiting);
 if(shots){await page.screenshot({path:join(shots,`time-lab${suffix}.png`)});await writeFile(join(shots,`time-lab${suffix}.json`),JSON.stringify({initial,first,before,stepped,result,sought,scrubbed,long,shorter,timing,corridor,delayed,closed,selected,band,tapped,painted,inspectedCell,playing,saved,reopened,shared,featured,inputs:receipts,paintTimes},null,2));}
}

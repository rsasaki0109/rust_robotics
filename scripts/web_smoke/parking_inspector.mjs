// Actual Hybrid A* samples, body geometry, gear changes and real pointer input.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';

const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([key])=>key.startsWith('parking'))));
const inputs=new WeakMap();
const point=(d,name)=>JSON.parse(d.parkingActions)[name];
const close=(a,b,label,tol=1e-8)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const wrap=(a)=>{const r=(a+Math.PI)%(2*Math.PI);return (r<0?r+2*Math.PI:r)-Math.PI;};
const corners=([x,y,a])=>[[-1,-0.95],[3.6,-0.95],[3.6,0.95],[-1,0.95]].map(([u,v])=>[x+u*Math.cos(a)-v*Math.sin(a),y+u*Math.sin(a)+v*Math.cos(a)]);
function overlaps(body,[x,y,w,h]) {
  const box=[[x,y],[x+w,y],[x+w,y+h],[x,y+h]];
  const axes=[[1,0],[0,1],...body.map((p,i)=>[p[1]-body[(i+1)%4][1],body[(i+1)%4][0]-p[0]])];
  return axes.every(([a,b])=>{
    const p=body.map(([x,y])=>a*x+b*y),q=box.map(([x,y])=>a*x+b*y);
    return Math.min(...p)<Math.max(...q)-1e-8&&Math.min(...q)<Math.max(...p)-1e-8;
  });
}
function geometry(d) {
  if(d.parkingActive!=='1')return;
  const p=JSON.parse(d.parkingPlan),i=Number(d.parkingCursor),origin=JSON.parse(d.parkingStart),pose=JSON.parse(d.parkingPose),goal=JSON.parse(d.parkingGoal);
  assert.deepEqual(JSON.parse(d.parkingPainted),[Number(d.parkingGeneration),i,d.parkingSweep==='1',d.parkingCircles==='1'],'map painted another sample');
  if(p) {
    const n=p.poses.length;assert(n>1);assert.equal(Number(d.parkingLast),n-1);assert(i>=0&&i<n);
    assert.equal(p.distance.length,n);assert.equal(p.directions.length,n);assert.deepEqual(p.poses[0],origin);
    assert.deepEqual(pose,p.poses[i],'inspection interpolated an unchecked pose');assert.equal(p.distance[0],0);
    assert(p.poses.every((s)=>s.every(Number.isFinite)));assert(p.directions.every((v)=>v===1||v===-1));
    p.poses.slice(1).forEach((s,j)=>close(p.distance[j+1]-p.distance[j],Math.hypot(s[0]-p.poses[j][0],s[1]-p.poses[j][1]),'distance from actual samples'));
    assert.deepEqual(p.changes,p.directions.flatMap((v,k)=>k&&v!==p.directions[k-1]?[k]:[]),'gear boundaries');
    const end=p.poses.at(-1);assert(Math.hypot(end[0]-goal[0],end[1]-goal[1])<0.6);assert(Math.abs(wrap(end[2]-goal[2]))<0.15);
    const cars=JSON.parse(d.parkingCars);
    for(const s of p.poses) {
      const b=corners(s);assert(b.every(([x,y])=>x>=-1e-8&&x<=30+1e-8&&y>=-1e-8&&y<=20+1e-8),'body left the lot');
      assert(!cars.some((car)=>overlaps(b,car)),'full rectangular body overlaps a parked car');
    }
    assert.equal(d.parkingFailure,'null');
  } else {assert.match(JSON.parse(d.parkingFailure),/collides/);assert.deepEqual(pose,origin);assert.equal(i,0);}
  const [x,y,w,h]=JSON.parse(d.parkingMap);close(w/h,1.5,'lot aspect',1e-5);
  const cover=JSON.parse(d.parkingCover),radius=Math.hypot(4.6/6,0.95);
  assert.equal(cover.length,3);cover.forEach(([offset,r],k)=>{close(offset,-1+(k+0.5)*4.6/3,'covering circle offset');close(r,radius,'covering circle radius');});
  const witnesses=JSON.parse(d.parkingWitnesses),ghosts=JSON.parse(d.parkingGhostIndices);
  assert.equal(witnesses.filter((v)=>v.kind==='body').length,4);
  assert.equal(witnesses.filter((v)=>v.kind==='switch').length,p?.changes.length??0);
  assert.equal(witnesses.filter((v)=>v.kind==='circle').length,d.parkingCircles==='1'?3:0);
  assert.equal(witnesses.filter((v)=>v.kind==='ghost').length,ghosts.length*4);
  if(d.parkingSweep==='0'||!p)assert.equal(ghosts.length,0);else assert(ghosts.length>0);
  for(const v of witnesses) {
    let expected;
    if(v.kind==='body') {assert.equal(v.index,i);expected=corners(pose)[v.corner];}
    else if(v.kind==='ghost') {assert(ghosts.includes(v.index));expected=corners(p.poses[v.index])[v.corner];}
    else if(v.kind==='switch') {assert(p.changes.includes(v.index));expected=p.poses[v.index].slice(0,2);}
    else if(v.kind==='circle') {assert.equal(v.index,i);const offset=cover[v.corner][0];expected=[pose[0]+offset*Math.cos(pose[2]),pose[1]+offset*Math.sin(pose[2])];}
    else assert.fail(`unknown painted geometry ${v.kind}`);
    expected.forEach((value,k)=>close(v.world[k],value,'painted body/circle/gear geometry'));
    close(v.pixel[0],x+w*v.world[0]/30,'painted x',0.0002);close(v.pixel[1],y+h-w*v.world[1]/30,'painted y',0.0002);
  }
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.parkingPointer ?? 'null');
      return Number(d?.parkingUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.parkingUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).parkingHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.parkingActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing parking control ${name}`);
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
      if (!d || Number(d.parkingUiRevision)<=revision) return false;
      const current=JSON.parse(d.parkingActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.parkingUiRevision),name,y,delta});
  }
  throw Error(`parking control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.parkingActionCount)>count,Number(before.parkingActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.parkingActive==='0' || JSON.stringify(JSON.parse(d.parkingPainted ?? 'null'))===JSON.stringify([Number(d.parkingGeneration),Number(d.parkingCursor),d.parkingSweep==='1',d.parkingCircles==='1']);
  });
  const after=await state(page);
  assert.equal(after.parkingLastAction,name);
  assert.equal(Number(after.parkingActionCount),Number(before.parkingActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.parkingActionCount),position,hit:JSON.parse(before.parkingHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}







async function graphInput(page,fraction,touch,drag=false) {
  for(let attempt=0;attempt<14;attempt++) {
    await visible(page,'graph');
    const d=await state(page),g=JSON.parse(d.parkingGraph),pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)];
    const start=pos(drag?0.2:fraction),end=pos(fraction);
    if(!await hover(page,'graph',start,false))continue;
    const before=await state(page);
    if(!JSON.parse(before.parkingGraph).every((v,k)=>v===g[k]))continue;
    const expected=Math.round((end[0]-g[0])/g[2]*Number(before.parkingLast));
    const reached=(n)=>Number(document.querySelector('canvas')?.dataset.parkingCursor)===n;
    if(drag&&touch) {
      const cdp=await page.context().newCDPSession(page);
      await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
      await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
      await page.waitForFunction(reached,expected);
      await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
    } else if(drag) {
      await page.mouse.move(...start);await page.mouse.down();await page.mouse.move(...end,{steps:4});
      await page.waitForFunction(reached,expected);await page.mouse.up();
    } else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
    const after=await acknowledged(page,'graph',before,end);
    assert.equal(Number(after.parkingCursor),expected);assert.equal(after.parkingPlaying,'0');return after;
  }
  throw Error('parking graph did not settle before input');
}

async function editGoal(page,touch,drag) {
  for(let attempt=0;attempt<14;attempt++) {
    await visible(page,'goal');
    const d=await state(page),r=JSON.parse(d.parkingMap),screen=([x,y])=>[Math.round(r[0]+x*r[2]/30),Math.round(r[1]+r[3]-y*r[2]/30)];
    const start=screen(drag?[21,9]:[23,8]),end=drag?screen([23,9.4]):start;
    if([...start,...end].some((v)=>!Number.isFinite(v)))assert.fail('invalid map transform');
    assert(start[1]>=64&&end[1]>=64&&start[1]<page.viewportSize().height-12&&end[1]<page.viewportSize().height-12,'goal gesture outside the visible scene');
    if(!await hover(page,'goal',start,false))continue;
    const before=await state(page);
    if(!JSON.parse(before.parkingMap).every((v,k)=>v===r[k]))continue;
    if(drag) {
      const pressed=({start,revision})=>{
        const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.parkingPress??'null');
        return Number(d?.parkingUiRevision)>revision&&d.parkingDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;
      };
      const processed=({end,revision})=>{
        const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.parkingPointer??'null');
        return Number(d?.parkingUiRevision)>revision&&d.parkingDrag!=='null'&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75;
      };
      const argument={end,revision:Number(before.parkingUiRevision)};
      if(touch) {
        const cdp=await page.context().newCDPSession(page);
        await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});
        // A whole gesture can fit between 2.5 Hz paints. Observe its actual
        // processed press origin before moving, then its real drag preview.
        await page.waitForFunction(pressed,{start,revision:Number(before.parkingUiRevision)});
        await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});
        await page.waitForFunction(processed,argument);
        await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();
      } else {
        await page.mouse.move(...start);await page.mouse.down();
        await page.waitForFunction(pressed,{start,revision:Number(before.parkingUiRevision)});
        await page.mouse.move(...end,{steps:4});
        await page.waitForFunction(processed,argument);await page.mouse.up();
      }
    } else if(touch)await page.touchscreen.tap(...start);else await page.mouse.click(...start);
    const after=await acknowledged(page,'goal',before,end),goal=JSON.parse(after.parkingGoal);
    close(goal[0],(start[0]-r[0])/r[2]*30,'goal x',0.00002);
    close(goal[1],(r[1]+r[3]-start[1])/r[2]*30,'goal y',0.00002);
    const heading=drag?Math.atan2(start[1]-end[1],end[0]-start[0]):JSON.parse(before.parkingGoal)[2];
    close(wrap(goal[2]-heading),0,'dragged goal heading');
    assert.equal(after.parkingStart,before.parkingStart,'view cursor changed the planning start');
    assert.equal(after.parkingCursor,'0');assert.equal(after.parkingPlaying,'0');assert.notEqual(after.parkingPlan,'null');
    return after;
  }
  throw Error('parking scene did not settle before goal input');
}

export async function checkParkingInspector(page,base,shots,touch=false,slow=false) {
  inputs.set(page,[]);
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.parkingActive==='1');
  const initial=await state(page);geometry(initial);assert.equal(initial.parkingCursor,'0');assert.equal(initial.parkingPlaying,'0');
  const p=JSON.parse(initial.parkingPlan),first=p.changes.find((i)=>p.distance[i]>1e-9)??p.changes[0];assert(first>0);
  const before=await action(page,'before',touch);assert.equal(Number(before.parkingCursor),first-1);
  const changed=await action(page,'step',touch);assert.equal(Number(changed.parkingCursor),first);
  assert.notEqual(p.directions[first],p.directions[first-1]);
  assert.equal(changed.parkingPlan,initial.parkingPlan);assert.equal(changed.parkingGeneration,initial.parkingGeneration,'stepping reran planning');
  const next=await action(page,'next',touch);assert.equal(Number(next.parkingCursor),p.changes.find((i)=>i>first)??p.poses.length-1);
  const previous=await action(page,'previous',touch);assert.equal(Number(previous.parkingCursor),p.changes.filter((i)=>i<Number(next.parkingCursor)).at(-1)??0);
  await action(page,'start',touch);assert.equal((await action(page,'back',touch)).parkingCursor,'0');
  const end=await action(page,'end',touch);assert.equal(end.parkingCursor,end.parkingLast);
  const ranked=await graphInput(page,0.3,touch),dragged=await graphInput(page,0.65,touch,true);
  assert.equal((await action(page,'sweep',touch)).parkingSweep,'0');await action(page,'sweep',touch);
  assert.equal((await action(page,'circles',touch)).parkingCircles,'1');await action(page,'circles',touch);
  await action(page,'start',touch);
  await action(page,'play',touch);
  await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.parkingCursor)>=3);
  const paused=await action(page,'play',touch);assert.equal(paused.parkingPlaying,'0');
  await page.waitForTimeout(900);const still=await state(page);
  assert.equal(still.parkingCursor,paused.parkingCursor);assert.equal(still.parkingPose,paused.parkingPose);assert.equal(still.parkingPlan,paused.parkingPlan,'paused replay changed path');
  const backIn=await action(page,'back_in',touch);assert.equal(backIn.parkingCursor,'0');assert(JSON.parse(backIn.parkingPlan).directions.includes(-1));
  const turn=await action(page,'turn',touch);assert.equal(turn.parkingCursor,'0');assert(Math.abs(Math.abs(JSON.parse(turn.parkingGoal)[2])-Math.PI)<1e-9);
  const blocked=await action(page,'blocked',touch);assert.equal(blocked.parkingPlan,'null');assert.match(JSON.parse(blocked.parkingFailure),/collides/);
  const reset=await action(page,'parallel',touch);assert.equal(reset.parkingPlan,initial.parkingPlan);
  const tapped=await editGoal(page,touch,false),aimed=await editGoal(page,touch,true);
  await action(page,'circles',touch);
  const selected=await graphInput(page,0.4,touch);
  assert.equal((await action(page,'live',touch)).parkingActive,'0');
  const inspected=await action(page,'inspect',touch);assert.equal(inspected.parkingPlan,selected.parkingPlan);assert.equal(inspected.parkingCursor,'0');
  const saved=await graphInput(page,0.35,touch);
  const receipts=inputs.get(page);assert.equal(receipts.length,27);assert(receipts.every((i)=>i.hit));
  let paintTimes;
  if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
  await page.goto(`${base}?${saved.parkingShare}`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.parkingActive==='1');
  const shared=await state(page);geometry(shared);
  for(const key of ['parkingCursor','parkingLast','parkingSweep','parkingCircles','parkingPlan','parkingStart','parkingGoal','parkingPose','parkingFailure'])assert.equal(shared[key],saved[key],`shared ${key} changed`);
  assert.equal(shared.parkingPlaying,'0');
  await page.goto(`${base}?tab=parking&parking_view=inspect&parking_event=before`,{waitUntil:'load'});
  await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.parkingActive==='1');
  const featured=await state(page);geometry(featured);
  assert.equal(Number(featured.parkingCursor),first-1);assert.equal(featured.parkingPlan,initial.parkingPlan);
  if(shots) {
    const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
    await page.screenshot({path:join(shots,`parking-lab${suffix}-feature.png`)});
    await writeFile(join(shots,`parking-lab${suffix}.json`),JSON.stringify({initial,before,changed,next,previous,end,ranked,dragged,paused,backIn,turn,blocked,reset,tapped,aimed,selected,inspected,saved,shared,featured,inputs:receipts,paintTimes},null,2));
  }
}

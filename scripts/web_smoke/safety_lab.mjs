// Actual CBF rollout decisions, independent rectangle distances and QP geometry.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const inputs=new WeakMap();
const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('safety'))));
const point=(d,key)=>JSON.parse(d.safetyActions)[key];
const close=(a,b,label,tol=1e-8)=>assert(Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1];
const distance=(a,b)=>Math.hypot(a[0]-b[0],a[1]-b[1]);
const clearance=(p,b)=>p[0]>=b[0]&&p[0]<=b[1]&&p[1]>=b[2]&&p[1]<=b[3]?-Math.min(p[0]-b[0],b[1]-p[0],p[1]-b[2],b[3]-p[1]):distance(p,[Math.max(b[0],Math.min(b[1],p[0])),Math.max(b[2],Math.min(b[3],p[1]))]);
function optimum(nominal,cs){
 const candidates=[nominal];for(const c of cs){const t=c.bound-dot(c.gradient,nominal);candidates.push(nominal.map((v,k)=>v+t*c.gradient[k]));}
 for(let i=0;i<cs.length;i++)for(let j=i+1;j<cs.length;j++){const a=cs[i],b=cs[j],det=a.gradient[0]*b.gradient[1]-a.gradient[1]*b.gradient[0];if(Math.abs(det)>1e-10)candidates.push([(a.bound*b.gradient[1]-a.gradient[1]*b.bound)/det,(a.gradient[0]*b.bound-a.bound*b.gradient[0])/det]);}
 return candidates.filter((u)=>cs.every((c)=>dot(c.gradient,u)>=c.bound-1e-7)).sort((a,b)=>distance(a,nominal)-distance(b,nominal))[0];
}
function geometry(d){
 if(d.safetyActive!=='1')return;
 const load=(k)=>JSON.parse(d['safety'+k]),runs=load('Runs'),boxes=load('Boxes'),goal=load('Goal'),cursor=Number(d.safetyCursor),selected=Number(d.safetySelected),radius=Number(d.safetyRadius),alpha=Number(d.safetyAlpha),margin=Number(d.safetyMargin),dt=Number(d.safetyDt),last=Number(d.safetyLast),cs=load('Constraints');
 assert.deepEqual(load('Painted'),[Number(d.safetyGeneration),cursor,selected,d.safetyFuture==='1']);assert.equal(runs.length,2);assert.equal(cs.length,boxes.length);assert.equal(last,Math.max(...runs.map((r)=>r.steps)));
 runs.forEach((r,run)=>{
  assert.equal(r.path.length,r.steps+1);assert.equal(r.clearances.length,r.path.length);assert.deepEqual(r.path[0],load('Start'));assert.deepEqual(r.position,r.path[Math.min(cursor,r.steps)]);
  r.path.forEach((p,i)=>{assert(p.every(Number.isFinite));close(r.clearances[i],Math.min(...boxes.map((b)=>clearance(p,b)))-radius,'disc clearance');});
  close(r.minimum,Math.min(...r.clearances),'whole-run minimum');close(r.observed_min,Math.min(...r.clearances.slice(0,Math.min(cursor,r.steps)+1)),'observed minimum');assert.equal(r.collided,r.minimum<0);
  assert.equal(Boolean(r.decision),cursor<r.steps);assert.equal(r.reached,distance(r.path.at(-1),goal)<=Number(d.safetyTolerance)&&r.steps<Number(d.safetyLimit));
  if(r.decision){const s=r.decision;assert.deepEqual(s.position,r.position);assert.deepEqual(s.next,r.path[cursor+1]);s.control.forEach((u,k)=>close(s.next[k],s.position[k]+u*dt,'actual Euler transition'));
   const target=goal.map((v,k)=>(v-s.position[k])*Number(d.safetyGain)),length=Math.hypot(...target),scale=Math.min(1,Number(d.safetySpeed)/length);target.forEach((v,k)=>close(s.nominal[k],v*scale,'nominal goal controller'));
   if(run===0){assert.deepEqual(s.control,s.nominal);assert.equal(s.intervened,false);}else assert.equal(s.intervened,distance(s.control,s.nominal)>1e-9);
  }
 });
 const current=runs[1],p=current.position,s=current.decision,nominal=s?.nominal??[0,0],applied=s?.control??[0,0];assert.equal(d.safetyHasStep==='1',Boolean(s));
 cs.forEach((c,i)=>{
  close(c.distance,clearance(p,boxes[i]),'signed rectangle distance');close(dot(c.gradient,c.gradient),1,'unit gradient');close(c.bound,-alpha*(c.distance-radius-margin),'CBF bound');close(c.nominal_slack,dot(c.gradient,nominal)-c.bound,'nominal slack');close(c.applied_slack,dot(c.gradient,applied)-c.bound,'applied slack');
  if(c.distance>1e-10){const b=boxes[i],q=[Math.max(b[0],Math.min(b[1],p[0])),Math.max(b[2],Math.min(b[3],p[1]))];c.gradient.forEach((v,k)=>close(v,(p[k]-q[k])/c.distance,'nearest-feature outward gradient'));}
 });
 if(s){const best=optimum(nominal,cs);if(best){assert(cs.every((c)=>c.applied_slack>=-1e-7),'applied velocity violates a feasible barrier');close(distance(applied,nominal),distance(best,nominal),'minimum correction',1e-7);}else assert(cs.some((c)=>c.applied_slack< -1e-7),'infeasible fallback was hidden');}
 const span=Number(d.safetySpan),feasible=load('Feasible');feasible.forEach((v)=>{assert(v.every((x)=>Math.abs(x)<=span+1e-7));assert(cs.every((c)=>dot(c.gradient,v)>=c.bound-1e-7));});
 const inside=(q)=>{if(feasible.length<3)return false;let sign=0;for(let i=0;i<feasible.length;i++){const a=feasible[i],b=feasible[(i+1)%feasible.length],cross=(b[0]-a[0])*(q[1]-a[1])-(b[1]-a[1])*(q[0]-a[0]);if(Math.abs(cross)>1e-7){if(sign&&Math.sign(cross)!==sign)return false;sign=Math.sign(cross);}}return true;};
 for(let i=-7;i<=7;i++)for(let j=-7;j<=7;j++){const q=[span*(i+0.17)/8,span*(j+0.23)/8],slacks=cs.map((c)=>dot(c.gradient,q)-c.bound);if(slacks.every((v)=>Math.abs(v)>1e-6))assert.equal(inside(q),slacks.every((v)=>v>=0),'clipped feasible region differs from all half-planes');}
 const [mx,my,mw,mh]=load('Map'),map=([x,y])=>[mx+(x+6)/12*mw,my+mh-(y+4)/8*mh];close(mh/mw,2/3,'map aspect',1e-6);
 const [vx,vy,vw,vh]=load('Velocity'),vel=([x,y])=>[vx+vw/2+x/span*vw/2,vy+vh/2-y/span*vh/2];close(vw,vh,'velocity square',1e-5);
 const [gx,gy,gw,gh]=load('Graph'),curves=load('Curves'),lo=Math.min(-0.25,...curves.flat()),hi=Math.max(0.3,...curves.flat()),curve=(i,v)=>[gx+i/Math.max(1,last)*gw,gy+gh-3-(v-lo)/(hi-lo)*(gh-6)];
 curves.forEach((values,r)=>{assert.equal(values.length,last+1);values.forEach((v,i)=>close(v,runs[r].clearances[Math.min(i,runs[r].steps)],'physical-time clearance curve'));});
 const ws=load('Witnesses'),of=(kind)=>ws.filter((v)=>v.kind===kind);assert.equal(of('robot').length,2);assert.equal(of('obstacle').length,boxes.length*4);assert.equal(of('clearance').length,(last+1)*2);assert.equal(of('velocity').length,s?2:0);assert.equal(of('command').length,s?2:0);assert.equal(of('correction').length,s?1:0);assert.equal(of('feasible').length,feasible.length);
 for(let r=0;r<2;r++)assert.equal(of('path').filter((v)=>v.run===r).length,d.safetyFuture==='1'?runs[r].path.length:Math.min(cursor,runs[r].steps)+1);
 for(const v of ws){
  if(v.kind==='clearance'){close(v.value,curves[v.run][v.index],'painted curve value');curve(v.index,v.value).forEach((a,k)=>close(v.pixel[k],a,'curve pixel',0.0003));continue;}
  if(v.kind==='cursor'){assert.equal(v.index,cursor);close(v.pixel[0],gx+cursor/Math.max(1,last)*gw,'time cursor',0.0003);close(v.pixel[1],gy+gh/2,'cursor y',0.0003);continue;}
  if(['velocity','feasible','boundary','correction'].includes(v.kind)){
   if(v.kind==='velocity')assert.deepEqual(v.velocity,v.role==='nominal'?nominal:applied);
   if(v.kind==='feasible')assert.deepEqual(v.velocity,feasible[v.index]);
   if(v.kind==='boundary')v.velocity.forEach((u)=>close(dot(cs[v.index].gradient,u),cs[v.index].bound,'constraint boundary'));
   if(v.kind==='correction')assert.deepEqual(v.velocity,[nominal,applied]);
   const many=['boundary','correction'].includes(v.kind);(many?v.velocity:[v.velocity]).forEach((q,i)=>vel(q).forEach((a,k)=>close((many?v.pixel[i]:v.pixel)[k],a,'velocity pixel',0.0003)));continue;
  }
  if(v.kind==='command'||v.kind==='distance'){
   assert.deepEqual(v.world[0],p);
   if(v.kind==='command'){const u=v.role==='nominal'?nominal:applied;p.forEach((a,k)=>close(v.world[1][k],a+u[k]*0.65,'world velocity arrow'));}
   else {const c=cs[v.index];p.forEach((a,k)=>close(v.world[1][k],a-c.gradient[k]*c.distance,'distance to nearest boundary'));}
   v.world.forEach((q,i)=>map(q).forEach((a,k)=>close(v.pixel[i][k],a,'world line pixel',0.0003)));continue;
  }
  let expected;
  if(v.kind==='robot'||v.kind==='margin'){expected=v.kind==='robot'?runs[v.run].position:p;close(v.radius,(radius+(v.kind==='margin'?margin:0))/12*mw,'disc pixel radius',0.0003);}
  else if(v.kind==='path'){expected=runs[v.run].path[v.index];assert.equal(v.future,v.index>Math.min(cursor,runs[v.run].steps));}
  else if(v.kind==='goal')expected=goal;
  else if(v.kind==='obstacle'){const b=boxes[v.index];expected=[[b[0],b[2]],[b[1],b[2]],[b[1],b[3]],[b[0],b[3]]][v.vertex];}
  else assert.fail(`unexpected safety witness ${v.kind}`);
  assert.deepEqual(v.world,expected);map(expected).forEach((a,k)=>close(v.pixel[k],a,'map pixel',0.0003));
 }
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.safetyPointer ?? 'null');
      return Number(d?.safetyUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.safetyUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).safetyHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.safetyActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing safety control ${name}`);
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
      if (!d || Number(d.safetyUiRevision)<=revision) return false;
      const current=JSON.parse(d.safetyActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.safetyUiRevision),name,y,delta});
  }
  throw Error(`safety control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.safetyActionCount)>count,Number(before.safetyActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.safetyActive==='0' || JSON.stringify(JSON.parse(d.safetyPainted ?? 'null'))===JSON.stringify([Number(d.safetyGeneration),Number(d.safetyCursor),Number(d.safetySelected),d.safetyFuture==='1']);
  });
  const after=await state(page);
  assert.equal(after.safetyLastAction,name);
  assert.equal(Number(after.safetyActionCount),Number(before.safetyActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.safetyActionCount),position,hit:JSON.parse(before.safetyHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function graphInput(page,fraction,touch,drag=false){
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,'graph');const d=await state(page),g=JSON.parse(d.safetyGraph),pos=(f)=>[Math.round(g[0]+f*g[2]),Math.round(g[1]+g[3]/2)],start=pos(drag?0.15:fraction),end=pos(fraction);
  if(!await hover(page,'graph',start,false))continue;const before=await state(page);if(!JSON.parse(before.safetyGraph).every((v,k)=>v===g[k]))continue;
  const expected=Math.round((end[0]-g[0])/g[2]*Number(before.safetyLast));
  if(drag){
   const pressed=({start,rev})=>{const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.safetyPress??'null');return Number(d?.safetyUiRevision)>rev&&d.safetyDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
   const reached=(i)=>Number(document.querySelector('canvas')?.dataset.safetyCursor)===i;
   if(touch){const cdp=await page.context().newCDPSession(page);await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await page.waitForFunction(pressed,{start,rev:Number(before.safetyUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});await page.waitForFunction(reached,expected);await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
   else{await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,rev:Number(before.safetyUiRevision)});await page.mouse.move(...end,{steps:4});await page.waitForFunction(reached,expected);await page.mouse.up();}
  }else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
  const after=await acknowledged(page,'graph',before,end);assert.equal(Number(after.safetyCursor),expected);assert.equal(after.safetyPlaying,'0');return after;
 }throw Error('safety chart did not settle before input');
}
async function editMap(page,touch,drag){
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,'map');const d=await state(page),r=JSON.parse(d.safetyMap),screen=([x,y])=>[Math.round(r[0]+(x+6)/12*r[2]),Math.round(r[1]+r[3]-(y+4)/8*r[3])],obstacle=d.safetyEdit==='obstacle';
  const start=screen(obstacle?[0.2,-1.6]:[2.8,-0.9]),end=drag?screen(obstacle?[1.,-1.5]:[3.4,-1.1]):start;
  assert(start[1]>=64&&end[1]>=64&&start[1]<page.viewportSize().height-12&&end[1]<page.viewportSize().height-12,'map gesture outside visible scene');
  if(!await hover(page,'map',start,false))continue;const before=await state(page);if(!JSON.parse(before.safetyMap).every((v,k)=>v===r[k]))continue;
  if(drag){
   const pressed=({start,rev})=>{const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.safetyPress??'null');return Number(d?.safetyUiRevision)>rev&&d.safetyDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
   const preview=({end,rev})=>{const d=document.querySelector('canvas')?.dataset,p=JSON.parse(d?.safetyPointer??'null');return Number(d?.safetyUiRevision)>rev&&d.safetyDrag!=='null'&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75;};
   if(touch){const cdp=await page.context().newCDPSession(page);await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await page.waitForFunction(pressed,{start,rev:Number(before.safetyUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});await page.waitForFunction(preview,{end,rev:Number(before.safetyUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
   else{await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,rev:Number(before.safetyUiRevision)});await page.mouse.move(...end,{steps:4});await page.waitForFunction(preview,{end,rev:Number(before.safetyUiRevision)});await page.mouse.up();}
  }else if(touch)await page.touchscreen.tap(...start);else await page.mouse.click(...start);
  const after=await acknowledged(page,'map',before,end),q=[(end[0]-r[0])/r[2]*12-6,(r[1]+r[3]-end[1])/r[3]*8-4];
  if(obstacle){const selected=Number(before.safetySelected),b=JSON.parse(before.safetyBoxes)[selected],a=JSON.parse(after.safetyBoxes)[selected];close((a[0]+a[1])/2,q[0],'obstacle centre x',2e-5);close((a[2]+a[3])/2,q[1],'obstacle centre y',2e-5);close(a[1]-a[0],b[1]-b[0],'preserved width');close(a[3]-a[2],b[3]-b[2],'preserved height');assert.equal(after.safetyGoal,before.safetyGoal);}
  else{JSON.parse(after.safetyGoal).forEach((v,k)=>close(v,q[k],'edited goal',2e-5));assert.equal(after.safetyBoxes,before.safetyBoxes);}
  assert.equal(after.safetyStart,before.safetyStart);assert.equal(after.safetyCursor,'0');assert.equal(after.safetyPlaying,'0');assert.equal(Number(after.safetyGeneration),Number(before.safetyGeneration)+1);return after;
 }throw Error('safety map did not settle before edit');
}
async function selectBoundary(page,touch){
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,'velocity');const d=await state(page),r=JSON.parse(d.safetyVelocity),span=Number(d.safetySpan),cs=JSON.parse(d.safetyConstraints),lines=JSON.parse(d.safetyWitnesses).filter((v)=>v.kind==='boundary'&&v.index!==Number(d.safetySelected));assert(lines.length,'no other boundary in the diagram');
  const line=lines[0],p=line.pixel[0].map((v,k)=>Math.round(v*0.35+line.pixel[1][k]*0.65));
  if(!await hover(page,'velocity',p,false))continue;const before=await state(page);if(!JSON.parse(before.safetyVelocity).every((v,k)=>v===r[k]))continue;
  const u=[(p[0]-r[0]-r[2]/2)/r[2]*2*span,(r[1]+r[3]/2-p[1])/r[3]*2*span],expected=cs.map((c,i)=>[i,Math.abs(dot(c.gradient,u)-c.bound)]).sort((a,b)=>a[1]-b[1])[0][0];
  if(touch)await page.touchscreen.tap(...p);else await page.mouse.click(...p);const after=await acknowledged(page,'velocity',before,p);assert.equal(Number(after.safetySelected),expected);assert.equal(after.safetyPlaying,'0');return after;
 }throw Error('safety velocity diagram did not settle before boundary input');
}
export async function checkSafetyLab(page,base,shots,touch=false,slow=false){
 inputs.set(page,[]);await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.safetyActive==='1');const initial=await state(page);geometry(initial);assert.equal(initial.safetyCursor,'0');assert.equal(initial.safetyPlaying,'0');
 const first=await action(page,'first',touch),at=Number(first.safetyFirst),a=JSON.parse(first.safetyRuns);assert.equal(Number(first.safetyCursor),at);assert(at>0);assert.deepEqual(a[0].position,a[1].position);assert(a[1].decision.intervened);assert.equal(a[1].observed_interventions,0);
 const stepped=await action(page,'step',touch),b=JSON.parse(stepped.safetyRuns);assert.equal(Number(stepped.safetyCursor),at+1);assert.notDeepEqual(b[0].position,b[1].position);assert.equal(b[1].observed_interventions,1);assert.equal(stepped.safetyGeneration,initial.safetyGeneration);
 await action(page,'back',touch);await action(page,'start',touch);const result=await action(page,'end',touch),r=JSON.parse(result.safetyRuns);assert(r[0].collided&&r[0].reached);assert(!r[1].collided&&r[1].reached);assert(r[1].minimum>0.09);
 const ranked=await graphInput(page,0.25,touch),scrubbed=await graphInput(page,0.6,touch,true);assert.equal((await action(page,'future',touch)).safetyFuture,'0');await action(page,'future',touch);
 const gentle=await action(page,'gentle',touch),firm=await action(page,'firm',touch),balanced=await action(page,'balanced',touch);assert.equal(gentle.safetyAlpha,'0.8');assert.equal(firm.safetyAlpha,'5');assert.equal(balanced.safetyRuns,initial.safetyRuns,'resetting gain changed deterministic solve');assert.deepEqual(JSON.parse(gentle.safetyRuns)[0].path,JSON.parse(firm.safetyRuns)[0].path);
 await action(page,'headon',touch);const headon=await action(page,'end',touch),h=JSON.parse(headon.safetyRuns)[1];assert(!h.collided&&!h.reached);assert.equal(h.steps,Number(headon.safetyLimit));assert(distance(h.path.at(-1),JSON.parse(headon.safetyGoal))>4);assert(distance(h.path.at(-1),h.path.at(-2))<1e-6);
 const gap=await action(page,'gap',touch);await action(page,'first',touch);const next=await action(page,'next_obstacle',touch);assert.equal(next.safetySelected,'1');const boundary=await selectBoundary(page,touch);assert.equal(boundary.safetySelected,'0');
 await action(page,'wide_margin',touch);const blocked=await action(page,'end',touch),w=JSON.parse(blocked.safetyRuns)[1];assert(!w.reached&&!w.collided);assert.deepEqual(JSON.parse(blocked.safetyRuns)[0].path,JSON.parse(gap.safetyRuns)[0].path);await action(page,'normal_margin',touch);
 const trapped=await action(page,'trapped',touch);assert.equal(trapped.safetyCursor,'0');assert(JSON.parse(trapped.safetyRuns)[1].clearances[0]<0);assert.deepEqual(JSON.parse(trapped.safetyFeasible),[]);assert(JSON.parse(trapped.safetyConstraints).some((c)=>c.applied_slack< -0.1));
 await action(page,'corner',touch);const playing=await action(page,'play',touch);assert.equal(playing.safetyPlaying,'1');await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.safetyCursor)>=3);
 const paused=await action(page,'play',touch);assert.equal(paused.safetyPlaying,'0');await page.waitForTimeout(900);const still=await state(page);assert.equal(still.safetyCursor,paused.safetyCursor);assert.equal(still.safetyRuns,paused.safetyRuns);assert.equal(still.safetyGeneration,paused.safetyGeneration);
 const tapped=await editMap(page,touch,false),aimed=await editMap(page,touch,true);await action(page,'gap',touch);await action(page,'next_obstacle',touch);await action(page,'edit_obstacle',touch);const moved=await editMap(page,touch,true);
 await action(page,'future',touch);const selected=await graphInput(page,0.3,touch);assert.equal((await action(page,'legacy',touch)).safetyActive,'0');const inspected=await action(page,'inspect',touch);assert.equal(inspected.safetyBoxes,moved.safetyBoxes);assert.equal(inspected.safetyFuture,'0');
 const saved=await graphInput(page,0.4,touch),receipts=inputs.get(page);assert.equal(receipts.length,36);assert(receipts.every((i)=>i.hit));let paintTimes;
 if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
 await page.goto(`${base}?${saved.safetyShare}`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.safetyActive==='1');const shared=await state(page);geometry(shared);
 for(const key of ['safetyCase','safetyStart','safetyGoal','safetyBoxes','safetyAlpha','safetyMargin','safetyCursor','safetySelected','safetyFuture','safetyEdit','safetyRuns','safetyConstraints','safetyFeasible'])assert.equal(shared[key],saved[key],`shared ${key} changed`);assert.equal(shared.safetyPlaying,'0');
 await page.route('https://fonts.googleapis.com/**',(route)=>route.fulfill({contentType:'text/css',body:''}));
 await page.goto(base.replace(/playground\/$/,''),{waitUntil:'load'});await page.locator('#play-safety-filter').click();await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.safetyActive==='1');const featured=await state(page);geometry(featured);assert.equal(featured.safetyCursor,featured.safetyFirst);assert(JSON.parse(featured.safetyRuns)[1].decision.intervened);
 if(shots){const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;await page.screenshot({path:join(shots,`safety-lab${suffix}-feature.png`)});await writeFile(join(shots,`safety-lab${suffix}.json`),JSON.stringify({initial,first,stepped,result,ranked,scrubbed,gentle,firm,balanced,headon,gap,boundary,blocked,trapped,playing,paused,tapped,aimed,moved,selected,inspected,saved,shared,featured,inputs:receipts,paintTimes},null,2));}
}

// Independent visibility, connected frontiers, queue distances and paint geometry.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const inputs=new WeakMap(),cache=new Map();
const state=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('explore'))));
const point=(d,key)=>JSON.parse(d.exploreActions)[key];
const close=(a,b,label,tol=0.0003)=>assert(Number.isFinite(a)&&Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const equal=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const W=24,H=16,N=W*H,idx=([x,y])=>y*W+x,cell=(i)=>[i%W,Math.floor(i/W)];
const compare=(a,b)=>a[0]-b[0]||a[1]-b[1];
const distance=(a,b)=>Math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2);
const neighbors=(p,diagonal=true)=>{
 const out=[];for(let x=-1;x<=1;x++)for(let y=-1;y<=1;y++){
  const q=[p[0]+x,p[1]+y];if((x||y)&&(diagonal||Math.abs(x)+Math.abs(y)===1)&&q[0]>=0&&q[1]>=0&&q[0]<W&&q[1]<H)out.push(q);
 }return out;
};
function rayVisible(occupied,a,b){
 const n=Math.max(1,3*Math.max(Math.abs(b[0]-a[0]),Math.abs(b[1]-a[1])));
 for(let i=1;i<=n;i++){
  const t=i/n,p=[Math.round(a[0]+t*(b[0]-a[0])),Math.round(a[1]+t*(b[1]-a[1]))];
  if(equal(p,b))return true;if(occupied(p))return false;
 }return true;
}
function observe(world,known,robot,range){
 for(let i=0;i<N;i++){const p=cell(i);if(distance(p,robot)<=range&&rayVisible((q)=>world[idx(q)],robot,p))known[i]=world[i]?2:1;}
}
function costs(known,source){
 const dist=Array(N).fill(Infinity),queue=[source],queued=Array(N).fill(false);dist[idx(source)]=0;queued[idx(source)]=true;
 for(let h=0;h<queue.length;h++){
  const p=queue[h];queued[idx(p)]=false;
  for(const q of neighbors(p)){const i=idx(q);if(known[i]!==1)continue;const next=dist[idx(p)]+distance(p,q);if(next<dist[i]){
   dist[i]=next;if(!queued[i]){queued[i]=true;queue.push(q);}
  }}
 }return dist;
}
function clusters(known){
 const unknown=(p)=>neighbors(p,false).filter((q)=>known[idx(q)]===0).length;
 const todo=new Set(Array.from({length:N},(_,i)=>i).filter((i)=>known[i]===1&&unknown(cell(i))>0)),result=new Map();
 while(todo.size){
  const seed=todo.values().next().value,stack=[seed],members=[];todo.delete(seed);
  while(stack.length){const i=stack.pop(),p=cell(i);members.push(p);for(const q of neighbors(p)){const j=idx(q);if(todo.delete(j))stack.push(j);}}
  const center=[0,1].map((k)=>members.reduce((v,p)=>v+p[k],0)/members.length),metric=(p)=>(p[0]-center[0])**2+(p[1]-center[1])**2;
  members.sort((a,b)=>metric(a)-metric(b)||compare(a,b));
  const rep=members[0];result.set(String(rep),{rep,members,openness:members.reduce((v,p)=>v+unknown(p),0)});
 }return result;
}
function rollout(d){
 const key=[d.exploreWorld,d.exploreStrategy,d.exploreRange,d.exploreBudget,d.exploreStart,d.exploreGoal].join('|');
 if(cache.has(key)){const r=cache.get(key);assert.equal(d.exploreSteps,r.serializedSteps);assert.equal(d.explorePath,r.serializedPath);return r;}
 const load=(k)=>JSON.parse(d['explore'+k]),world=[...d.exploreWorld].map((b)=>b==='1'),path=load('Path'),steps=load('Steps'),weights=load('Weights'),start=load('Start'),goal=load('Goal'),range=Number(d.exploreRange),budget=Number(d.exploreBudget);
 assert.equal(world.length,N);assert.equal(d.exploreWidth,'24');assert.equal(d.exploreHeight,'16');assert.equal(d.exploreLimit,'64');assert([3,5,7].includes(range));assert([1,3].includes(budget));assert.deepEqual(path[0],start);assert(path.length>=1&&steps.length>=1);
 assert.deepEqual(weights,d.exploreStrategy==='balanced'?[1,.35,1.5,.15]:d.exploreStrategy==='curious'?[.3,.35,1.5,5]:[2,.35,1.5,0]);
 const known=Array(N).fill(0),lengths=[0];observe(world,known,start,range);
 for(let at=0;at<steps.length;at++){
  const s=steps[at],next=steps[at+1];assert.deepEqual(s.robot,path[s.at]);assert.deepEqual(s.known,known,'actual sensing boundary');
  if(s.phase==='frontier'){
   const cs=clusters(known),max=Math.max(1,...[...cs.values()].map((c)=>c.openness)),dists=costs(known,s.robot);
   assert.equal(s.candidates.length,cs.size);assert(s.chosen!==null&&s.chosen<s.candidates.length);
   for(const c of s.candidates){
    const expected=cs.get(String(c.rep));assert(expected);assert.equal(new Set(c.members.map(String)).size,c.members.length);
    assert.deepEqual([...c.members].sort(compare),[...expected.members].sort(compare));assert.equal(c.openness,expected.openness);close(c.normalized,c.openness/max,'normalized discovery',1e-9);
    const gain=distance(s.robot,goal)-distance(c.rep,goal),cost=dists[idx(c.rep)],los=rayVisible((p)=>known[idx(p)]===2,s.robot,c.rep);
    close(c.gain,gain,'goal gain',1e-9);assert.equal(c.los,los);
    if(Number.isFinite(cost)){close(c.cost,cost,'known-free travel',1e-9);close(c.score,weights[0]*gain-weights[1]*cost+weights[2]*(los?1:0)+weights[3]*c.openness/max,'frontier score',1e-9);}
    else{assert.equal(c.cost,null);assert.equal(c.score,null);}
   }
   const chosen=s.candidates[s.chosen];assert.equal(chosen.score,Math.max(...s.candidates.filter((c)=>c.score!==null).map((c)=>c.score)));
   s.candidates.forEach((c)=>{if(c.score===chosen.score)assert(compare(chosen.rep,c.rep)<=0,'actual score tie break');});
  }else assert.equal(s.candidates.length,0);
  if(next){
   assert(['frontier','goal'].includes(s.phase));assert(next.at>=s.at);
   const target=s.phase==='goal'?goal:s.candidates[s.chosen].rep;
   if(s.phase==='goal')assert.equal(known[idx(goal)],1);
   const dists=costs(known,target);
   for(let t=s.at+1;t<=next.at;t++){
    const a=path[t-1],b=path[t];assert(neighbors(a).some((q)=>equal(q,b)));assert.equal(known[idx(b)],1,'entered unknown space');assert.equal(world[idx(b)],false);
    const minimum=Math.min(...neighbors(a).filter((p)=>known[idx(p)]===1).map((p)=>dists[idx(p)]));
    close(dists[idx(b)],minimum,'gradient step',1e-9);assert(dists[idx(b)]<dists[idx(a)]);lengths.push(lengths.at(-1)+distance(a,b));
   }
   if(s.phase==='frontier'&&next.at>s.at)observe(world,known,next.robot,range);
  }
 }
 const last=steps.at(-1);assert.equal(last.at,path.length-1);assert.equal(d.exploreReached==='1',last.phase==='arrived');close(Number(d.exploreLength),lengths.at(-1),'recorded length',1e-9);
 if(last.phase==='arrived')assert.deepEqual(path.at(-1),goal);
 else if(last.phase==='no_frontiers')assert.equal(clusters(known).size,0);
 else if(last.phase==='iteration_limit')assert.equal(d.exploreIterations,'64');
 else if(last.phase==='no_progress')assert.equal(steps.at(-2).at,last.at);
 else assert.equal(last.phase,'unreachable');
 const coverage=path.map((_,t)=>steps.findLast((s)=>s.at<=t).known.filter((k)=>k!==0).length);
 const firstWall=steps.find((s)=>s.phase==='frontier'&&s.known.some((v,i)=>v===2&&cell(i)[0]>0&&cell(i)[0]<W-1&&cell(i)[1]>0&&cell(i)[1]<H-1))?.at??0;
 const result={world,path,steps,weights,start,goal,range,budget,lengths,coverage,firstWall,serializedSteps:d.exploreSteps,serializedPath:d.explorePath};cache.set(key,result);return result;
}
function geometry(d){
 if(d.exploreActive!=='1')return;
 const r=rollout(d),load=(k)=>JSON.parse(d['explore'+k]),t=Number(d.exploreCursor),di=r.steps.findLastIndex((s)=>s.at<=t),s=r.steps[di],selected=Number(d.exploreSelected),truth=d.exploreTruth==='1'||d.exploreEdit==='walls',end=r.steps[di+1]?.at??t;
 assert.equal(Number(d.exploreLast),r.path.length-1);assert.equal(Number(d.exploreDecision),di);assert.deepEqual(load('Known'),s.known);assert.deepEqual(load('Robot'),r.path[t]);assert.equal(d.explorePhase,s.phase);assert.deepEqual(load('Coverage'),r.coverage);assert.equal(Number(d.exploreFirstWall),r.firstWall);
 load('Lengths').forEach((v,i)=>close(v,r.lengths[i],'prefix length',1e-9));assert(selected<=Math.max(0,s.candidates.length-1));
 const [mx,my,mw,mh]=load('Map'),[gx,gy,gw,gh]=load('Graph'),[cx,cy,cw,ch]=load('Candidates'),[tx,ty,tw,th]=load('Components'),side=mw/W,screen=(p)=>[mx+(p[0]+.5)*side,my+(p[1]+.5)*side],project=(at)=>[gx+at/Math.max(1,r.path.length-1)*gw,gy+gh-r.coverage[at]/N*gh];
 close(mh/H,side,'square cells');
 const ws=load('Witnesses'),of=(kind)=>ws.filter((w)=>w.kind===kind);
 assert.equal(of('cell').length,N);assert.equal(new Set(of('cell').map((w)=>String(w.cell))).size,N);assert.equal(of('frontier').length,s.candidates.reduce((v,c)=>v+c.members.length,0));assert.equal(of('representative').length,s.candidates.length);
 assert.equal(of('history').length,t+1);assert.equal(of('plan').length,d.exploreRoute==='1'&&end>t?end-t+1:0);assert.equal(of('coverage').length,t+1);assert.equal(of('cursor').length,1);assert.equal(of('robot').length,1);assert.equal(of('sensor').length,1);assert.equal(of('goal').length,1);assert.equal(of('score').length,s.candidates.length);
 const candidate=s.candidates[selected],values=candidate?.cost!==null&&candidate?[
  r.weights[0]*candidate.gain,-r.weights[1]*candidate.cost,r.weights[2]*(candidate.los?1:0),r.weights[3]*candidate.normalized]:null;
 assert.equal(of('component').length,values?4:0);
 for(const w of ws){
  if(w.kind==='cell'){
   const i=idx(w.cell),p=screen(w.cell);assert.equal(w.known,s.known[i]);assert.equal(w.occupied,r.world[i]);assert.equal(w.truth,truth);close(w.pixel[0],p[0]-side/2,'cell x');close(w.pixel[1],p[1]-side/2,'cell y');close(w.size,side,'cell size');
   assert.deepEqual(w.rgb,w.known===1?[53,77,94]:w.known===2?[161,172,184]:truth&&r.world[i]?[76,53,63]:[21,30,43]);
  }else if(w.kind==='frontier'||w.kind==='representative'){
   const c=s.candidates[w.candidate];assert(c.members.some((p)=>equal(p,w.cell)));
   const p=screen(w.cell);
   if(w.kind==='representative'){assert.deepEqual(w.cell,c.rep);p.forEach((v,k)=>close(w.pixel[k],v,'representative'));close(w.radius,side*.24,'representative radius');}
   else{assert.equal(w.chosen,w.candidate===s.chosen);assert.equal(w.selected,w.candidate===selected);p.forEach((v,k)=>close(w.pixel[k],v-side/2+1,'frontier outline'));close(w.size,side-2,'frontier size');}
  }else if(['history','plan','robot','sensor','goal'].includes(w.kind)){
   const c=w.kind==='robot'?r.path[t]:w.kind==='sensor'?s.robot:w.kind==='goal'?r.goal:r.path[w.at];assert.deepEqual(w.cell,c);screen(c).forEach((v,k)=>close(w.pixel[k],v,'map point'));
   if(w.kind==='plan')assert(w.at>=t&&w.at<=end);if(w.kind==='robot')close(w.radius,side*.31,'robot radius');if(w.kind==='sensor')close(w.radius,r.range*side,'last sensor range');if(w.kind==='goal')close(w.size,side*.55,'goal size');
  }else if(w.kind==='coverage'||w.kind==='cursor'){
   assert(w.at<=t);if(w.kind==='coverage')assert.equal(w.count,r.coverage[w.at]);else assert.equal(w.at,t);project(w.at).forEach((v,k)=>close(w.pixel[k],v,'coverage paint'));
  }else if(w.kind==='score'||w.kind==='component'){
   const score=w.kind==='score',v=score?s.candidates[w.candidate].score:values[w.component],all=score?s.candidates.map((c)=>Math.abs(c.score??0)):values.map(Math.abs),scale=Math.max(1,...all),rect=score?[cx,cy,cw,ch]:[tx,ty,tw,th],row=score?w.candidate:w.component,n=score?s.candidates.length:4;
   assert.equal(w.value,v);close(w.scale,scale,'bar domain',1e-9);const zero=rect[0]+rect[2]/2,width=(v??0)/scale*rect[2]*.45,y=rect[1]+(row+.5)/n*rect[3];[Math.min(zero,zero+width),y-5,Math.abs(width),10].forEach((a,k)=>close(w.pixel[k],a,'score bar'));
  }else assert.fail(`unexpected exploration witness ${w.kind}`);
 }
 assert.deepEqual(load('Painted'),[Number(d.exploreGeneration),t,selected,d.exploreTruth==='1',d.exploreRoute==='1',d.exploreEdit]);
}

async function hover(page, name, position, followWidget = true) {
  for (const dx of [-2, 0]) {
    const before = await state(page), p = [Math.floor(position[0] + dx), Math.floor(position[1])];
    await page.mouse.move(...p);
    await page.waitForFunction(({ revision, p }) => {
      const d=document.querySelector('canvas')?.dataset, pointer=JSON.parse(d?.explorePointer ?? 'null');
      return Number(d?.exploreUiRevision)>revision && pointer && Math.hypot(pointer[0]-p[0],pointer[1]-p[1])<0.75;
    }, { revision:Number(before.exploreUiRevision), p });
    const current=point(await state(page),name);
    if (followWidget && Math.hypot(current[0]-position[0],current[1]-position[1])>1) return false;
  }
  return JSON.parse((await state(page)).exploreHits)[name] === true;
}

async function visible(page, name) {
  await page.waitForFunction((name)=>Boolean(JSON.parse(document.querySelector('canvas')?.dataset.exploreActions ?? '{}')[name]),name);
  for (let i=0;i<14;i++) {
    const d=await state(page), p=point(d,name), {width,height}=page.viewportSize();
    assert(p, `missing explore control ${name}`);
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
      if (!d || Number(d.exploreUiRevision)<=revision) return false;
      const current=JSON.parse(d.exploreActions)[name][1];
      return delta<0 ? current>y+1 : current<y-1;
    },{revision:Number(latest.exploreUiRevision),name,y,delta});
  }
  throw Error(`explore control unreachable: ${name}`);
}

async function acknowledged(page, name, before, position) {
  await page.waitForFunction((count)=>Number(document.querySelector('canvas')?.dataset.exploreActionCount)>count,Number(before.exploreActionCount));
  await page.waitForFunction(()=>{
    const d=document.querySelector('canvas').dataset;
    return d.exploreActive==='0' || JSON.stringify(JSON.parse(d.explorePainted ?? 'null'))===JSON.stringify([Number(d.exploreGeneration),Number(d.exploreCursor),Number(d.exploreSelected),d.exploreTruth==='1',d.exploreRoute==='1',d.exploreEdit]);
  });
  const after=await state(page);
  assert.equal(after.exploreLastAction,name);
  assert.equal(Number(after.exploreActionCount),Number(before.exploreActionCount)+1,`${name} processed twice`);
  geometry(after);
  inputs.get(page).push({name,count:Number(after.exploreActionCount),position,hit:JSON.parse(before.exploreHits)[name]});
  return after;
}

async function action(page, name, touch) {
  const p=await visible(page,name), before=await state(page);
  if (touch) await page.touchscreen.tap(...p); else await page.mouse.click(...p);
  return acknowledged(page,name,before,p);
}

async function regionInput(page,key,from,to,touch,drag=false){
 const attribute='explore'+key[0].toUpperCase()+key.slice(1);
 for(let attempt=0;attempt<14;attempt++){
  await visible(page,key);const d=await state(page),r=JSON.parse(d[attribute]);
  const screen=(q)=>key==='map'?[Math.round(r[0]+(q[0]+0.5)/Number(d.exploreWidth)*r[2]),Math.round(r[1]+(q[1]+0.5)/Number(d.exploreHeight)*r[3])]:[Math.round(r[0]+q*r[2]),Math.round(r[1]+r[3]/2)];
  const start=screen(from),end=screen(to),height=page.viewportSize().height;
  if(Math.min(start[1],end[1])<70||Math.max(start[1],end[1])>height-24){
   await page.mouse.move(Math.round(r[0]+r[2]/2),height-60);
   const before=await state(page),y=JSON.parse(before[attribute])[1],delta=Math.min(start[1],end[1])<70?-200:200;
   await page.mouse.wheel(0,delta);
   await page.waitForFunction(({attribute,y,revision})=>{const d=document.querySelector('canvas').dataset;return Number(d.exploreUiRevision)>revision&&Math.abs(JSON.parse(d[attribute])[1]-y)>1;},{attribute,y,revision:Number(before.exploreUiRevision)});
   continue;
  }
  if(!await hover(page,key,start,false))continue;
  const before=await state(page);if(!equal(JSON.parse(before[attribute]),r))continue;
  const expected=key==='map'?null:Math.min(Number(d.exploreLast),Math.round((end[0]-r[0])/r[2]*Number(d.exploreLast)));
  if(drag){
   const pressed=({start,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.explorePress??'null');return Number(d.exploreUiRevision)>revision&&d.exploreDown==='1'&&p&&Math.hypot(p[0]-start[0],p[1]-start[1])<0.75;};
   const moved=({end,key,expected,revision})=>{const d=document.querySelector('canvas').dataset,p=JSON.parse(d.explorePointer??'null');return Number(d.exploreUiRevision)>revision&&p&&Math.hypot(p[0]-end[0],p[1]-end[1])<0.75&&(key==='map'?d.exploreDrag!=='null':Number(d.exploreCursor)===expected);};
   if(touch){const cdp=await page.context().newCDPSession(page);await cdp.send('Input.dispatchTouchEvent',{type:'touchStart',touchPoints:[{x:start[0],y:start[1]}]});await page.waitForFunction(pressed,{start,revision:Number(before.exploreUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchMove',touchPoints:[{x:end[0],y:end[1]}]});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.exploreUiRevision)});await cdp.send('Input.dispatchTouchEvent',{type:'touchEnd',touchPoints:[]});await cdp.detach();}
   else{await page.mouse.move(...start);await page.mouse.down();await page.waitForFunction(pressed,{start,revision:Number(before.exploreUiRevision)});await page.mouse.move(...end,{steps:4});await page.waitForFunction(moved,{end,key,expected,revision:Number(before.exploreUiRevision)});await page.mouse.up();}
  }else if(touch)await page.touchscreen.tap(...end);else await page.mouse.click(...end);
  const after=await acknowledged(page,key,before,end);
  if(key!=='map'){assert.equal(Number(after.exploreCursor),expected);assert.equal(after.explorePlaying,'0');}
  return after;
 }throw Error(`explore ${key} did not settle before input`);
}

export async function checkExplorationLab(page,base,shots,touch=false,slow=false){
 inputs.set(page,[]);
 await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.exploreActive==='1');
 const initial=await state(page);geometry(initial);assert.equal(initial.exploreCursor,'0');assert.equal(initial.explorePlaying,'0');
 const stepped=await action(page,'step',touch);assert.equal(stepped.exploreCursor,'1');assert.equal(stepped.exploreGeneration,initial.exploreGeneration);
 const observed=await action(page,'next_decision',touch);assert(Number(observed.exploreCursor)>1);assert.equal(JSON.parse(observed.exploreSteps)[Number(observed.exploreDecision)].at,Number(observed.exploreCursor));
 const result=await action(page,'end',touch);assert.equal(result.explorePhase,'arrived');assert.deepEqual(JSON.parse(result.exploreRobot),JSON.parse(result.exploreGoal));
 const sought=await regionInput(page,'graph',.4,.4,touch),scrubbed=await regionInput(page,'graph',.2,.6,touch,true);
 const revealed=await action(page,'truth',touch);assert.equal(revealed.exploreTruth,'1');assert.equal(revealed.exploreSteps,scrubbed.exploreSteps);assert.equal(revealed.exploreKnown,scrubbed.exploreKnown);
 await action(page,'truth',touch);const hiddenPlan=await action(page,'route',touch);assert.equal(hiddenPlan.exploreRoute,'0');await action(page,'route',touch);
 await action(page,'edit_goal',touch);const movedGoal=await regionInput(page,'map',[21,9],[21,9],touch);assert.deepEqual(JSON.parse(movedGoal.exploreGoal),[21,9]);assert.equal(movedGoal.exploreCursor,'0');assert.equal(Number(movedGoal.exploreGeneration),Number(scrubbed.exploreGeneration)+1);
 const doors=await action(page,'doors',touch);assert.equal(doors.exploreReached,'1');
 const curious=await action(page,'curious',touch);assert.equal(curious.exploreStrategy,'curious');assert.deepEqual(JSON.parse(curious.exploreWeights),[.3,.35,1.5,5]);
 await action(page,'goal_strategy',touch);const balanced=await action(page,'balanced',touch);assert.equal(balanced.exploreSteps,doors.exploreSteps);
 const short=await action(page,'range_short',touch);assert.equal(short.exploreRange,'3');
 const long=await action(page,'range_long',touch);assert.equal(long.exploreRange,'7');
 const normal=await action(page,'range_normal',touch);assert.equal(normal.exploreSteps,doors.exploreSteps);
 const frequent=await action(page,'budget_one',touch);assert.equal(frequent.exploreBudget,'1');const resumedBudget=await action(page,'budget_three',touch);assert.equal(resumedBudget.exploreSteps,doors.exploreSteps);
 const sealed=await action(page,'sealed',touch);assert.equal(sealed.exploreReached,'0');assert.equal(JSON.parse(sealed.exploreSteps).at(-1).phase,'no_frontiers');assert.notDeepEqual(JSON.parse(sealed.explorePath).at(-1),JSON.parse(sealed.exploreGoal));
 const hall=await action(page,'hall',touch);assert.equal(hall.exploreReached,'1');const corner=await action(page,'corner',touch);assert.equal(corner.exploreReached,'1');
 await action(page,'inspect',touch);const choices=JSON.parse(corner.exploreSteps)[0].candidates,target=choices.at(-1).rep;
 const inspected=await regionInput(page,'map',target,target,touch);assert.equal(Number(inspected.exploreSelected),choices.length-1);assert.equal(inspected.exploreGeneration,corner.exploreGeneration);
 await action(page,'walls',touch);const painted=await regionInput(page,'map',[12,13],[13,13],touch,true);assert.equal(painted.exploreCursor,'0');assert.equal(Number(painted.exploreGeneration),Number(corner.exploreGeneration)+1);assert.equal(painted.exploreWorld[13*24+12],'1');assert.equal(painted.exploreWorld[13*24+13],'1');assert.notEqual(painted.exploreWorld,corner.exploreWorld);
 await action(page,'inspect',touch);const playing=await action(page,'play',touch);assert.equal(playing.explorePlaying,'1');
 await page.waitForFunction(()=>Number(document.querySelector('canvas').dataset.exploreCursor)>=3);
 const saved=await action(page,'play',touch);assert.equal(saved.explorePlaying,'0');assert(Number(saved.exploreCursor)>=3);assert.equal(saved.exploreGeneration,painted.exploreGeneration);
 assert.equal((await action(page,'legacy',touch)).exploreActive,'0');const reopened=await action(page,'explore_lab',touch);assert.equal(reopened.exploreActive,'1');
 const keys=['exploreCase','exploreStrategy','exploreWeights','exploreRange','exploreBudget','exploreWorld','exploreStart','exploreGoal','explorePath','exploreSteps','exploreCursor','exploreSelected','exploreKnown','exploreDecision','exploreRobot','explorePhase','exploreCoverage','exploreLengths','exploreTruth','exploreRoute','exploreEdit'];
 for(const key of [...keys,'exploreGeneration'])assert.equal(reopened[key],saved[key],`returning changed ${key}`);
 const receipts=inputs.get(page);assert.equal(receipts.length,32);assert(receipts.every((r)=>r.hit));assert.deepEqual(receipts.map((r)=>r.count),Array.from({length:32},(_,i)=>i+1));
 let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'slow paints were not capped');}
 await page.goto(`${base}?${saved.exploreShare}`,{waitUntil:'load'});await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.exploreActive==='1');const shared=await state(page);geometry(shared);
 for(const key of keys)assert.equal(shared[key],saved[key],`shared ${key} changed`);assert.equal(shared.explorePlaying,'0');
 const suffix=`${touch?'-phone':''}${slow?'-slow':''}${page.viewportSize().width===900?'-compact':''}`;
 if(shots)await page.screenshot({path:join(shots,`exploration-lab${suffix}-shared.png`)});
 await page.route('https://fonts.googleapis.com/**',(route)=>route.fulfill({contentType:'text/css',body:''}));
 await page.goto(base.replace(/playground\/$/,''),{waitUntil:'load'});await page.locator('#play-exploration').click();
 await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.exploreActive==='1');const featured=await state(page);geometry(featured);assert.equal(featured.exploreCase,'corner');assert.equal(featured.exploreCursor,featured.exploreFirstWall);assert(Number(featured.exploreCursor)>0);assert.equal(featured.exploreTruth,'0');assert.equal(featured.explorePlaying,'0');
 if(shots){await page.screenshot({path:join(shots,`exploration-lab${suffix}.png`)});await writeFile(join(shots,`exploration-lab${suffix}.json`),JSON.stringify({initial,stepped,observed,result,sought,scrubbed,revealed,hiddenPlan,movedGoal,doors,curious,balanced,short,long,normal,frequent,resumedBudget,sealed,hall,corner,inspected,painted,playing,saved,reopened,shared,featured,inputs:receipts,paintTimes},null,2));}
}

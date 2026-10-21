// Independently evaluate the real sample lattice, costs, winner and rollout.
// All mutations use a real mouse/touch input on an acknowledged widget.
import assert from 'node:assert/strict';
import { writeFile } from 'node:fs/promises';
import { join } from 'node:path';
const read=(page)=>page.locator('canvas').evaluate((c)=>Object.fromEntries(Object.entries(c.dataset).filter(([k])=>k.startsWith('dwa'))));
const load=(d,k)=>JSON.parse(d['dwa'+k]);
const close=(a,b,label,tol=1e-8)=>assert(Number.isFinite(a)&&Math.abs(a-b)<tol,`${label}: ${a} != ${b}`);
const near=(a,b,label,tol=1e-8)=>{assert.equal(a.length,b.length);a.forEach((v,i)=>close(v,b[i],label,tol));};
const caches=new Map(),receipts=new WeakMap();
const round=(v)=>Math.sign(v)*Math.floor(Math.abs(v)+.5);
const advance=(s,[v,w])=>{const yaw=s[2]+w*.2;return[s[0]+v*Math.cos(yaw)*.2,s[1]+v*Math.sin(yaw)*.2,yaw,v,w];};
function obstacles(d,at){
 const result=[];for(let i=0;i<=40;i++){result.push([i*.3,0],[i*.3,8]);}for(let i=1;i<27;i++){result.push([0,i*.3],[12,i*.3]);}
 if(d.dwaCase==='wall')for(let i=0;i<18;i++)result.push([6,.6+i*.3]);
 if(d.dwaCase==='crossing')result.push([6,4+2*Math.sin(at*.2*1.1)]);
 result.push(load(d,'Probe'));return result;
}
function evaluate(d,s,at){
 const [goal,weights]=[load(d,'Goal'),load(d,'Weights')],radius=Number(d.dwaRadius),horizon=Number(d.dwaHorizon),obs=obstacles(d,at);
 const window=[Math.max(0,s[3]-.2),Math.min(1.2,s[3]+.2),Math.max(-1.2,s[4]-.4),Math.min(1.2,s[4]+.4)];
 const samples=(lo,hi)=>Array.from({length:Math.max(0,Math.ceil((hi-lo)/.1))},(_,i)=>round((lo+.1*i)/.1)*.1);
 const cs=[];let chosen=null,min=Number.MAX_VALUE;
 for(const v of samples(...window.slice(0,2)))for(const w of samples(...window.slice(2))){
  const control=[v,w],states=[[...s]];let t=0;while(t<=horizon){states.push(advance(states.at(-1),control));t+=.2;}
  const end=states.at(-1),angle=Math.atan2(goal[1]-end[1],goal[0]-end[0])-end[2],heading=weights[0]*Math.abs(Math.atan2(Math.sin(angle),Math.cos(angle))),speed=weights[1]*(1.2-v);
  let nearest=Infinity;for(const p of states)for(const o of obs)nearest=Math.min(nearest,Math.sqrt((p[0]-o[0])**2+(p[1]-o[1])**2));
  const obstacle=nearest<=radius?null:weights[2]/nearest,total=obstacle===null?null:heading+speed+obstacle;
  cs.push({control,states,goal:heading,speed,obstacle,total});if(total!==null&&total<=min){min=total;chosen=cs.length-1;}
 }
 const applied=chosen===null?[0,0]:[...cs[chosen].control];const override=chosen!==null&&Math.abs(applied[0])<.001&&Math.abs(s[3])<.001;if(override)applied[1]=-2;
 return{window,cs,chosen,applied,override};
}
function rollout(d){
 const key=[d.dwaCase,d.dwaPolicy,d.dwaHorizon,d.dwaRadius,d.dwaGoal,d.dwaProbe].join('|');
 if(caches.has(key)){const r=caches.get(key);assert.equal(d.dwaPath,r.pathJSON);assert.equal(d.dwaControls,r.controlsJSON);return r;}
 const goal=load(d,'Goal'),path=[[2,4,0,.6,0]],controls=[];
 for(let at=0;at<120;at++){const s=path.at(-1);if(Math.hypot(s[0]-goal[0],s[1]-goal[1])<=.4)break;const e=evaluate(d,s,at);controls.push(e.applied);path.push(advance(s,e.applied));}
 const actual=load(d,'Path'),commands=load(d,'Controls');assert.equal(actual.length,path.length);actual.forEach((s,i)=>near(s,path[i],'executed state',1e-7));assert.equal(commands.length,controls.length);commands.forEach((s,i)=>near(s,controls[i],'actual command',1e-7));
 const result={path,controls,pathJSON:d.dwaPath,controlsJSON:d.dwaControls};caches.set(key,result);return result;
}
function verify(d){
 if(d.dwaActive!=='1')return;
 const at=Number(d.dwaCursor),selected=Number(d.dwaSelected),r=rollout(d),s=load(d,'State'),e=evaluate(d,s,at),cs=load(d,'Candidates'),goal=load(d,'Goal');
 assert.equal(d.dwaDt,'0.2');assert.equal(d.dwaLimit,'120');assert.equal(Number(d.dwaLast),r.path.length-1);near(s,r.path[at],'selected pose',1e-7);
 assert.deepEqual(load(d,'Weights'),d.dwaPolicy==='clearance'?[1.5,1,3]:d.dwaPolicy==='speed'?[1.5,3,.3]:[1.5,1,1]);
 near(load(d,'Window'),e.window,'window');assert.equal(load(d,'Chosen'),e.chosen);near(load(d,'Applied'),e.applied,'applied');assert.equal(d.dwaOverride==='1',e.override);assert.equal(cs.length,e.cs.length);
 cs.forEach((c,i)=>{const expected=e.cs[i];near(c.control,expected.control,'sample');assert.equal(c.states.length,expected.states.length);c.states.forEach((p,j)=>near(p,expected.states[j],'predicted state'));for(const term of ['goal','speed','obstacle','total']){if(expected[term]===null)assert.equal(c[term],null);else close(c[term],expected[term],term);}});
 const obs=load(d,'Obstacles');const expectedObs=obstacles(d,at);assert.equal(obs.length,expectedObs.length);obs.forEach((p,i)=>near(p,expectedObs[i],'point obstacle'));
 const reached=Math.hypot(s[0]-goal[0],s[1]-goal[1])<=.4;assert.equal(d.dwaStatus,reached?'arrived':at===120?'limit':e.chosen===null?'blocked':'planning');
 const [x,y,w,h]=load(d,'Map'),[vx,vy,vw,vh]=load(d,'Velocity'),[bx,by,bw]=load(d,'Bars');assert(Math.abs(h-w*2/3)<=1/64+1e-6,'map aspect differs beyond egui layout quantization (1/64 point)');
 const map=(p)=>[x+p[0]/12*w,y+h-p[1]/8*h],vel=(p)=>[vx+(p[0]+.1)/1.4*vw,vy+vh/2-p[1]/4.4*vh];
 const ws=load(d,'Witnesses'),of=(k)=>ws.filter((v)=>v.kind===k);assert.equal(of('obstacle').length,obs.length);assert.equal(of('sample').length,cs.length);assert.equal(of('robot').length,1);assert.equal(of('goal').length,1);assert.equal(of('history').length,1);assert.equal(of('window').length,1);assert.equal(of('cost').length,cs[selected]?3:0);assert.equal(of('applied').length,at<r.path.length-1?1:0);
 const arcs=cs.map((_,i)=>i).filter((i)=>d.dwaCloud==='1'||i===e.chosen||i===selected);assert.deepEqual(of('arc').map((v)=>v.index),arcs);
 for(const witness of ws){
  if(witness.kind==='obstacle'){near(witness.world,obs[witness.index],'obstacle position');near(witness.pixel,map(witness.world),'obstacle paint',.0005);}
  else if(witness.kind==='arc'){assert.equal(witness.chosen,witness.index===e.chosen);assert.equal(witness.selected,witness.index===selected);assert.equal(witness.pixels.length,cs[witness.index].states.length);witness.pixels.forEach((p,j)=>near(p,map(cs[witness.index].states[j]),'arc paint',.0005));}
  else if(witness.kind==='sample'){near(witness.pixel,vel(cs[witness.index].control),'sample paint',.0005);assert.equal(witness.chosen,witness.index===e.chosen);assert.equal(witness.selected,witness.index===selected);}
  else if(witness.kind==='robot'){near(witness.state,s,'painted state');near(witness.pixel,map(s),'robot paint',.0005);close(witness.radius,Number(d.dwaRadius)*w/12,'footprint',.0005);near(witness.heading,map([s[0]+Number(d.dwaRadius)*Math.cos(s[2]),s[1]+Number(d.dwaRadius)*Math.sin(s[2])]),'heading',.0005);}
  else if(witness.kind==='goal')near(witness.pixel,map(goal),'goal paint',.0005);
  else if(witness.kind==='history'){assert.equal(witness.pixels.length,at+1);witness.pixels.forEach((p,i)=>near(p,map(r.path[i]),'prefix history',.0005));}
  else if(witness.kind==='applied'){near(witness.state,r.path[at+1],'one-step applied',1e-7);near(witness.pixel,map(r.path[at+1]),'one-step paint',.0005);}
  else if(witness.kind==='window'){const valid=e.window[0]<e.window[1]&&e.window[2]<e.window[3];assert.equal(witness.valid,valid);if(valid){const a=vel([e.window[0],e.window[2]]),b=vel([e.window[1],e.window[3]]);near(witness.rect,[Math.min(a[0],b[0]),Math.min(a[1],b[1]),Math.abs(a[0]-b[0]),Math.abs(a[1]-b[1])],'window rectangle',.0005);}else{assert.equal(witness.rect,null);assert.equal(cs.length,0);}}
  else if(witness.kind==='cost'){const c=cs[selected],values=[c.goal,c.speed,c.obstacle],scale=Math.max(1,...values.filter((v)=>v!==null)),value=values[witness.term];assert.equal(witness.value,value);close(witness.scale,scale,'bar scale');near(witness.rect,[bx,by+witness.term*24+16,(value??0)/scale*bw,5],'cost paint',.0005);}
  else assert.fail(`unknown DWA witness ${witness.kind}`);
 }
 assert.deepEqual(load(d,'Painted'),[Number(d.dwaGeneration),at,selected,Number(d.dwaCloud)]);
}
async function idle(page){await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.dwaScrolling==='0');}
async function hover(page,name,target,followWidget=true){
 for(const dx of [-2,0]){const d=await read(page),p=[Math.floor(target[0]+dx),Math.floor(target[1])];await page.mouse.move(...p);await page.waitForFunction(({rev,p})=>{const d=document.querySelector('canvas')?.dataset,q=JSON.parse(d?.dwaPointer??'null');return Number(d?.dwaUiRevision)>rev&&d.dwaScrolling==='0'&&q&&Math.hypot(q[0]-p[0],q[1]-p[1])<.75;},{rev:Number(d.dwaUiRevision),p});const current=load(await read(page),'Actions')[name];if(followWidget&&Math.hypot(current[0]-target[0],current[1]-target[1])>1)return false;}
 return load(await read(page),'Hits')[name]===true;
}
async function wheel(page,name,delta){
 const d=await read(page),before=load(d,'Actions')[name][1];await page.mouse.wheel(0,delta);await page.waitForFunction(({rev,scroll,name,before,delta})=>{const d=document.querySelector('canvas')?.dataset;if(!d||Number(d.dwaUiRevision)<=rev||Number(d.dwaScrollRevision)<=scroll||d.dwaScrolling!=='0')return false;const y=JSON.parse(d.dwaActions)[name][1];return delta<0?y>before+1:y<before-1;},{rev:Number(d.dwaUiRevision),scroll:Number(d.dwaScrollRevision),name,before,delta});
}
async function visible(page,name,relative){
 await idle(page);
 for(let n=0;n<18;n++){
  const d=await read(page),a=load(d,'Actions')[name];assert(a,`missing DWA control ${name}`);const v=page.viewportSize();
  const p=relative?relative(d):a;
  if(p[0]>=0&&p[0]<=v.width&&p[1]>=66&&p[1]<=v.height-26){if(await hover(page,name,p,!relative)){const fresh=relative?relative(await read(page)):load(await read(page),'Actions')[name];if(Math.hypot(fresh[0]-p[0],fresh[1]-p[1])<=1)return p;}continue;}
  await page.mouse.move(Math.min(v.width-20,Math.max(20,a[0])),v.height-60);
  await wheel(page,name,p[1]<66?-240:240);
 }throw Error(`DWA widget unreachable ${name}`);
}
async function act(page,name,touch,relative){
 const p=await visible(page,name,relative),before=await read(page);assert.equal(load(before,'Hits')[name],true);assert.equal(before.dwaScrolling,'0');
 if(touch)await page.touchscreen.tap(...p);else await page.mouse.click(...p);
 await page.waitForFunction((n)=>Number(document.querySelector('canvas')?.dataset.dwaActionCount)>n,Number(before.dwaActionCount));
 await page.waitForFunction(()=>{const d=document.querySelector('canvas')?.dataset;return d&&d.dwaActive==='1'&&JSON.stringify(JSON.parse(d.dwaPainted??'null'))===JSON.stringify([Number(d.dwaGeneration),Number(d.dwaCursor),Number(d.dwaSelected),Number(d.dwaCloud)]);});
 const after=await read(page);assert.equal(after.dwaLastAction,name);assert.equal(Number(after.dwaActionCount),Number(before.dwaActionCount)+1);verify(after);receipts.get(page).push({name,count:Number(after.dwaActionCount),position:p,hit:load(before,'Hits')[name],scrolling:before.dwaScrolling});return after;
}
const stable=(d)=>Object.fromEntries(['dwaShare','dwaState','dwaCandidates','dwaObstacles','dwaWindow','dwaPath','dwaControls','dwaStatus'].map((k)=>[k,d[k]]));
export async function checkDwaLab(page,base,shots,touch=false,slow=false){
 receipts.set(page,[]);await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.dwaActive==='1');const initial=await read(page);verify(initial);assert.equal(initial.dwaCursor,'0');assert.equal(initial.dwaPlaying,'0');
 let d=await act(page,'step',touch);assert.equal(d.dwaCursor,'1');assert.equal(d.dwaGeneration,initial.dwaGeneration);
 d=await act(page,'back',touch);assert.equal(d.dwaCursor,'0');
 d=await act(page,'next',touch);assert.equal(Number(d.dwaSelected),Math.min(Number(initial.dwaSelected)+1,load(d,'Candidates').length-1));
 await act(page,'chosen',touch);await act(page,'cloud',touch);d=await act(page,'cloud',touch);assert.equal(d.dwaCloud,'1');
 for(const [name,policy]of[['clearance','clearance'],['speed','speed'],['balanced','balanced']]){d=await act(page,name,touch);assert.equal(d.dwaPolicy,policy);assert.equal(d.dwaCursor,'0');}
 for(const [name,value]of[['short','1.6'],['long','3.2'],['medium','2.4']]){d=await act(page,name,touch);assert.equal(d.dwaHorizon,value);}
 for(const [name,value]of[['small','0.25'],['large','0.7'],['normal','0.45']]){d=await act(page,name,touch);assert.equal(d.dwaRadius,value);}
 d=await act(page,'crossing',touch);assert.equal(d.dwaCase,'crossing');const first=load(d,'Obstacles').at(-2);d=await act(page,'step',touch);assert.notDeepEqual(load(d,'Obstacles').at(-2),first);
 d=await act(page,'timeline',touch,(d)=>{const[x,y,w,h]=load(d,'Timeline');return[Math.round(x+w*.25),Math.round(y+h/2)];});assert.equal(Number(d.dwaCursor),Math.round(Number(d.dwaLast)*.25));
 d=await act(page,'velocity',touch,(d)=>{const[x,y,w,h]=load(d,'Velocity'),[v,omega]=load(d,'Candidates')[0].control;return[Math.round(x+(v+.1)/1.4*w),Math.round(y+h/2-omega/4.4*h)];});assert.equal(d.dwaSelected,'0');
 await act(page,'edit_goal',touch);d=await act(page,'map',touch,(d)=>{const[x,y,w,h]=load(d,'Map');return[Math.round(x+w*.8),Math.round(y+h*.3)];});assert.equal(d.dwaCursor,'0');const changedGoal=load(d,'Goal');assert(Math.hypot(changedGoal[0]-9.6,changedGoal[1]-5.6)<.1);
 await act(page,'edit_obstacle',touch);d=await act(page,'map',touch,(d)=>{const[x,y,w,h]=load(d,'Map');return[Math.round(x+w*.4),Math.round(y+h*.7)];});assert(Math.hypot(load(d,'Probe')[0]-4.8,load(d,'Probe')[1]-2.4)<.1);
 await act(page,'inspect_arc',touch);await act(page,'step',touch);await act(page,'next',touch);await act(page,'cloud',touch);
 const edits=stable(await read(page)),visitor=await page.context().browser().newContext({viewport:page.viewportSize(),hasTouch:touch,deviceScaleFactor:touch?2:1});let shared;
 try{const p=await visitor.newPage(),errors=[];p.on('pageerror',(e)=>errors.push(e.message));await p.goto(`${base}?${edits.dwaShare}`,{waitUntil:'load'});await p.waitForFunction(()=>document.querySelector('canvas')?.dataset.dwaActive==='1');shared=await read(p);verify(shared);assert.equal(shared.dwaPlaying,'0');assert.deepEqual(stable(shared),edits);assert.deepEqual(errors,[]);}finally{await visitor.close();}
 d=await act(page,'trapped',touch);assert.equal(d.dwaStatus,'blocked');assert.equal(d.dwaChosen,'null');assert(load(d,'Candidates').every((c)=>c.total===null));await act(page,'collision',touch);d=await act(page,'step',touch);near(load(d,'State').slice(0,3),[2,4,0],'blocked fallback did not drive');
 d=await act(page,'end',touch);assert.equal(d.dwaStatus,'limit');
 d=await act(page,'open',touch);assert.equal(d.dwaCase,'open');await act(page,'play',touch);await page.waitForFunction(()=>Number(document.querySelector('canvas')?.dataset.dwaCursor)>=3);d=await act(page,'play',touch);assert.equal(d.dwaPlaying,'0');const paused=stable(d);await page.waitForTimeout(slow?900:300);assert.deepEqual(stable(await read(page)),paused);
 d=await act(page,'wall',touch);const empty=await act(page,'timeline',touch,(d)=>{const[x,y,w,h]=load(d,'Timeline');return[Math.round(x+w*30/Number(d.dwaLast)),Math.round(y+h/2)];});assert.equal(empty.dwaCursor,'30');assert.equal(empty.dwaCandidates,'[]');assert.equal(empty.dwaChosen,'null');assert(load(empty,'Window')[2]>load(empty,'Window')[3]);assert.equal(load(empty,'Witnesses').find((w)=>w.kind==='window').rect,null);await act(page,'timeline',touch,(d)=>{const[x,y,w,h]=load(d,'Timeline');return[Math.round(x+w*8/Number(d.dwaLast)),Math.round(y+h/2)];});await visible(page,'map');verify(d=await read(page));
 let paintTimes;if(slow){paintTimes=await page.evaluate(()=>window.__slowPaintTimes);assert(paintTimes.length>=3&&paintTimes.slice(1).every((t,i)=>t-paintTimes[i]>=350),'actual paints exceed 2.5 Hz');}
 if(shots){const suffix=slow?'-slow':touch?'-phone':page.viewportSize().width===900?'-compact':'';await page.screenshot({path:join(shots,`dwa-lab${suffix}.png`)});await writeFile(join(shots,`dwa-lab${suffix}.json`),JSON.stringify({initial,edited:edits,shared,final:d,empty,inputs:receipts.get(page),paintTimes},null,2));}
 await page.route('https://fonts.googleapis.com/**',(route)=>route.fulfill({contentType:'text/css',body:''}));await page.goto(new URL('../',base).href,{waitUntil:'load'});await page.locator('#play-dwa').click();await page.waitForFunction(()=>document.querySelector('canvas')?.dataset.dwaActive==='1');const home=await read(page);verify(home);assert.equal(home.dwaCase,'wall');assert.equal(home.dwaCursor,'8');assert.equal(home.dwaPlaying,'0');
}

/* Local, progressively enhanced reader. All research data are bundled; no fetch. */
(() => {
  'use strict';
  const $ = s => document.querySelector(s), $$ = s => [...document.querySelectorAll(s)];
  const data = window.VECR_DATA;
  const escape = value => String(value).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const fixed = (value, digits=2) => value === '' || value === null || value === undefined ? 'Undefined' : Number(value).toLocaleString('en-US', {minimumFractionDigits:digits, maximumFractionDigits:digits});
  const intervals = raw => raw ? raw.split(';').filter(Boolean).map(x => '[' + x.split(':').map(v => fixed(v, 3)).join(', ') + '] m').join(' ∪ ') : 'none';
  const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)');
  let toastTimer;
  const toast = text => { $('#toast').textContent=text; $('#toast').classList.add('visible'); clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('#toast').classList.remove('visible'),2200); };
  function setTheme(mode) {
    document.documentElement.dataset.theme=mode;
    $('#themeToggle').setAttribute('aria-label',`Switch to ${mode==='light'?'dark':'light'} theme`);
    try { localStorage.setItem('vecr-reader-theme',mode); } catch (_) { /* Preferences are optional offline. */ }
  }
  try { const saved=localStorage.getItem('vecr-reader-theme'); if (saved==='dark'||saved==='light') setTheme(saved); } catch (_) {}
  $('#themeToggle').addEventListener('click',()=>setTheme(document.documentElement.dataset.theme==='dark'?'light':'dark'));
  $('#printPage').addEventListener('click',()=>window.print());
  let scale=1;
  function changeFont(delta) { scale=Math.max(.9,Math.min(1.3,scale+delta));document.documentElement.style.setProperty('--reading-scale',scale);toast(`Reading text: ${Math.round(scale*100)}%`); }
  $('#fontIncrease').addEventListener('click',()=>changeFont(.1));$('#fontDecrease').addEventListener('click',()=>changeFont(-.1));
  const mobile=$('#mobileToc'), sidebar=$('#sidebar');
  function closeNav(){sidebar.classList.remove('open');mobile.setAttribute('aria-expanded','false');mobile.setAttribute('aria-label','Open chapter navigation');}
  mobile.addEventListener('click',()=>{const open=sidebar.classList.toggle('open');mobile.setAttribute('aria-expanded',String(open));mobile.setAttribute('aria-label',`${open?'Close':'Open'} chapter navigation`);});
  $$('.chapter-nav a').forEach(a=>a.addEventListener('click',closeNav));
  const headings=$$('.chapter>h2'), nav=$$('.chapter-nav a');
  let scrollQueued=false;
  function updateScroll(){scrollQueued=false;const h=document.documentElement.scrollHeight-innerHeight;$('#readProgress').style.width=(h>0?100*scrollY/h:0)+'%';let active='';for(const item of headings){if(item.getBoundingClientRect().top<=150)active=item.id;else break;}nav.forEach(a=>{const on=a.hash==='#'+active;a.classList.toggle('active',on);if(on)a.setAttribute('aria-current','location');else a.removeAttribute('aria-current');});}
  addEventListener('scroll',()=>{if(!scrollQueued){scrollQueued=true;requestAnimationFrame(updateScroll);}},{passive:true});updateScroll();
  const search=$('#chapterSearch'), results=$('#searchResults');
  const searchable=$$('.chapter h2,.chapter h3,.chapter h4,.chapter>p,.chapter>ul>li,.chapter>ol>li');
  search.addEventListener('input',()=>{const q=search.value.trim().toLocaleLowerCase();if(q.length<2){results.hidden=true;return;}let target='overview';const hits=[];for(const e of searchable){if(e.id)target=e.id;const text=e.textContent.trim(),at=text.toLocaleLowerCase().indexOf(q);if(at>=0)hits.push({target,text:text.slice(Math.max(0,at-25),Math.min(text.length,at+130)),heading:/^H/.test(e.tagName)});}hits.sort((a,b)=>Number(b.heading)-Number(a.heading));results.innerHTML=hits.length?hits.slice(0,16).map(x=>`<a href="#${escape(x.target)}"><strong>${x.heading?'SECTION':'TEXT MATCH'}</strong>${escape(x.text)}</a>`).join(''):'<p>No matching dissertation text.</p>';results.hidden=false;});
  results.addEventListener('click',e=>{if(e.target.closest('a')){results.hidden=true;closeNav();}});
  const dialog=$('#figureDialog');
  $$('.figure-open').forEach(button=>button.addEventListener('click',()=>{$('#figureImage').src=button.dataset.image;$('#figureImage').alt=button.dataset.caption;$('#figureCaption').textContent=button.dataset.caption;dialog.showModal();}));
  $('#closeFigure').addEventListener('click',()=>dialog.close());
  dialog.addEventListener('click',e=>{if(e.target===dialog){const r=dialog.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)dialog.close();}});
  document.addEventListener('keydown',e=>{if(e.key==='Escape'){closeNav();results.hidden=true;}if((e.ctrlKey||e.metaKey)&&e.key.toLowerCase()==='k'){e.preventDefault();if(innerWidth<=720){sidebar.classList.add('open');mobile.setAttribute('aria-expanded','true');}search.focus();search.select();}});
  // Expanding disclosures before print makes the defense answers complete on paper.
  let printOpen=[];
  addEventListener('beforeprint',()=>{printOpen=$$('details').map(e=>e.open);$$('details').forEach(e=>e.open=true);});
  addEventListener('afterprint',()=>$$('details').forEach((e,i)=>e.open=printOpen[i]||false));
  if(!data)return;
  // The compact replay contains verbatim rows from five frozen v8 cases.
  const slider=$('#replaySlider'), play=$('#playReplay');
  let current='N07',playing=false,raf=0,lastTick=0,replayTime=100,eventItems=[],bounds=null;
  const ns='http://www.w3.org/2000/svg';
  function elt(tag,attrs={},text){const n=document.createElementNS(ns,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));if(text!==undefined)n.textContent=text;return n;}
  function stop(){playing=false;cancelAnimationFrame(raf);raf=0;play.textContent='Play replay';}
  function sample(trace,time){let lo=0,hi=trace.length;while(lo<hi){const m=(lo+hi)>>1;if(Number(trace[m].time_ms)<=time)lo=m+1;else hi=m;}return trace[Math.max(0,lo-1)];}
  function coordinate(x,y){return [65+(x-bounds.x0)/(bounds.x1-bounds.x0)*760,320-(y-bounds.y0)/(bounds.y1-bounds.y0)*270];}
  // The following spatial strips are reconstructed exclusively from each
  // recorded row's exact missing intervals. The scalar physical-truth score is
  // *not* projected into an invented spatial map.
  function acceptedSegments(raw) {
    const missing=(raw||'').split(';').filter(Boolean).map(pair=>pair.split(':').map(Number))
      .filter(([a,b])=>Number.isFinite(a)&&Number.isFinite(b)&&b>a)
      .map(([a,b])=>[Math.max(0,Math.min(40,a)),Math.max(0,Math.min(40,b))]).sort((a,b)=>a[0]-b[0]);
    const segments=[];let cursor=0;
    for(const [a,b] of missing){if(a>cursor)segments.push([cursor,a]);cursor=Math.max(cursor,b);}
    if(cursor<40)segments.push([cursor,40]);return segments;
  }
  function draw(time){
    const target=$('#replayDrawing');target.replaceChildren();
    const [xa,ya]=coordinate(0,4),[xb,yb]=coordinate(40,-4);
    target.append(elt('rect',{x:xa,y:ya,width:xb-xa,height:yb-ya,rx:6,fill:'#e1eade'}));
    // Coordinates use the same independent, metric east/north samples as the
    // original frozen replay. The highlighted ±4 m band is illustrative.
    for(let x=0;x<=40;x+=5){const [px]=coordinate(x,0);target.append(elt('line',{x1:px,y1:32,x2:px,y2:326,stroke:'#d6ded2','stroke-width':x%10===0?1.4:.7}));if(x%10===0)target.append(elt('text',{x:px,y:346,'text-anchor':'middle',fill:'#54685c','font-size':12},`${x} m`));}
    for(let y=Math.ceil(bounds.y0/4)*4;y<=bounds.y1;y+=4){const [,py]=coordinate(0,y);target.append(elt('line',{x1:65,y1:py,x2:825,y2:py,stroke:'#e3e9df'}));target.append(elt('text',{x:54,y:py+4,'text-anchor':'end',fill:'#54685c','font-size':11},`${y}`));}
    target.append(elt('line',{x1:xa,y1:coordinate(0,0)[1],x2:xb,y2:coordinate(0,0)[1],stroke:'#779990','stroke-dasharray':'8 7','stroke-width':1.8}));
    target.append(elt('text',{x:65,y:21,fill:'#365548','font-size':12},'Recorded 2D physical traces · shaded corridor ±4 m'));
    target.append(elt('text',{x:825,y:365,'text-anchor':'end',fill:'#54685c','font-size':11},'Along-edge position (m)'));
    ['B2','VECR'].forEach(policy=>{
      const run=data.replays[current][policy],trace=run.trace,color=policy==='VECR'?'#147562':'#ad672e';
      const rows=trace.filter(r=>Number(r.time_ms)<=time),row=sample(trace,time);
      const fullPath=trace.map(r=>coordinate(Number(r.east_m),Number(r.north_m)).join(',')).join(' ');
      target.append(elt('polyline',{points:fullPath,fill:'none',stroke:color,'stroke-opacity':.17,'stroke-width':2.5}));
      const traveled=rows.map(r=>coordinate(Number(r.east_m),Number(r.north_m)).join(',')).join(' ');
      if(traveled)target.append(elt('polyline',{points:traveled,fill:'none',stroke:color,'stroke-width':3.3,'stroke-linejoin':'round','stroke-linecap':'round'}));
      // Two parallel bars encode the *accepted* interval union, not assumed truth.
      const bandY=policy==='B2'?5.7:-5.7;
      const [bx1,by]=coordinate(0,bandY),[bx2]=coordinate(40,bandY);
      target.append(elt('line',{x1:bx1,y1:by,x2:bx2,y2:by,stroke:color,'stroke-opacity':.20,'stroke-width':7,'stroke-linecap':'round'}));
      acceptedSegments(row.missing).forEach(([a,b])=>{const [left]=coordinate(a,bandY),[right]=coordinate(b,bandY);target.append(elt('line',{x1:left,y1:by,x2:right,y2:by,stroke:color,'stroke-opacity':.85,'stroke-width':7,'stroke-linecap':'round'}));});
      target.append(elt('text',{x:816,y:by-10,'text-anchor':'end','font-size':11,fill:color},policy==='B2'?'B2 accepted':'VECR accepted'));
      const [x,y]=coordinate(Number(row.east_m),Number(row.north_m));
      const prev=rows.length>=2?rows[rows.length-2]:row;
      const dx=Number(row.east_m)-Number(prev.east_m),dy=Number(row.north_m)-Number(prev.north_m);
      const heading=Math.abs(dx)+Math.abs(dy)<1e-6?0:Math.atan2(dy,dx)*180/Math.PI;
      // A top-down vehicle glyph conveys heading; no fictitious physical attitude.
      const body=elt('g',{transform:`translate(${x} ${y}) rotate(${heading})`});
      if(Math.abs(Number(row.north_m))>4)body.append(elt('circle',{r:17,fill:'#dd7041','fill-opacity':.12,stroke:'#d87d4e','stroke-width':1.5,'stroke-dasharray':'3 3'}));
      body.append(elt('path',{d:'M-10-10L10 10M-10 10L10-10',stroke:color,'stroke-width':2.2}));
      [[-11,-11],[11,-11],[-11,11],[11,11]].forEach(([cx,cy])=>body.append(elt('circle',{cx,cy,r:4.8,fill:'#f5f6f0',stroke:color,'stroke-width':1.8})));
      body.append(elt('path',{d:'M13 0L-5 -7L-3 0L-5 7Z',fill:color,stroke:'#ffffff','stroke-width':.5}));
      target.append(body);
      const commands=run.events.filter(e=>e.kind==='goto_accepted'&&Number(e.time_ms)<=time);
      if(commands.length){const last=commands[commands.length-1],[tx,ty]=coordinate(Number(last.target_east),Number(last.target_north));target.append(elt('path',{d:`M${tx-6} ${ty-6}L${tx+6} ${ty+6}M${tx+6} ${ty-6}L${tx-6} ${ty+6}`,stroke:color,'stroke-width':2.5}));}
    });
  }

  function render(time){replayTime=Math.min(Number(slider.max),Math.max(100,time));slider.value=String(Math.round(replayTime/100)*100);const t=Number(slider.value);$('#replayTime').textContent=`${fixed(t/1000,1)} / ${fixed(Number(slider.max)/1000,1)} s`;slider.setAttribute('aria-valuetext',`${fixed(t/1000,1)} simulation seconds`);draw(t);$('#replayMetrics').innerHTML=['B2','VECR'].map(policy=>{const trace=data.replays[current][policy].trace,r=sample(trace,t),accepted=Math.max(0,Math.min(100,Number(r.reported_covered_m)/40*100)),truth=Math.max(0,Math.min(100,Number(r.truth_covered_m)/40*100)),ended=t>=Number(trace.at(-1).time_ms);return `<div class="replay-card"><h4><span class="${policy==='VECR'?'legend-vecr':'legend-b2'}">${policy==='VECR'?'VECR':'B2'}</span><span class="phase">${escape(r.phase)}</span></h4><p>${escape(r.reason)}${ended?' · final recorded state':''}</p><p>Travel ${fixed(r.distance_m)} m · repair count ${r.recoveries}<br>Bounded arrival: ${r.at_b_verified==='1'?'yes':'no'} · actual stopped B: ${r.truth_at_b==='1'?'yes':'no'}</p><div class="coverage-label"><span>Agent-accepted strip</span><strong>${fixed(accepted,1)}%</strong></div><div class="coverage-track"><span style="width:${accepted}%"></span></div><div class="coverage-label"><span>Actual acquired full strip</span><strong>${fixed(truth,1)}%</strong></div><div class="coverage-track truth"><span style="width:${truth}%"></span></div><p title="Exact recorded intervals: ${escape(r.missing||'none')}">Missing: ${escape(intervals(r.missing))}<br>Sample time ${fixed(Number(r.time_ms)/1000,1)} s</p></div>`;}).join('');$$('.event-button').forEach(b=>b.classList.toggle('current',Number(b.dataset.time)<=t&&Number(b.dataset.time)>t-500));}
  function loadCase(id){stop();current=id;$('#replayCase').value=id;const traces=Object.values(data.replays[id]).flatMap(x=>x.trace);const xs=traces.map(x=>Number(x.east_m)),ys=traces.map(x=>Number(x.north_m));bounds={x0:Math.min(-2,...xs)-1,x1:Math.max(42,...xs)+1,y0:Math.min(-6,...ys)-1,y1:Math.max(10,...ys)+1};slider.max=String(Math.max(...traces.map(x=>Number(x.time_ms))));eventItems=['B2','VECR'].flatMap(policy=>data.replays[id][policy].events.filter(e=>['phase','goto_accepted','goto_rejected','motor_failure','battery_low'].includes(e.kind)).map(e=>({...e,policy}))).sort((a,b)=>Number(a.time_ms)-Number(b.time_ms)||a.policy.localeCompare(b.policy));$('#eventList').innerHTML=eventItems.map(e=>`<button class="event-button" data-time="${e.time_ms}" aria-label="Inspect ${escape(e.policy)} ${escape(e.kind)} at ${Number(e.time_ms)/1000} seconds"><time>${fixed(Number(e.time_ms)/1000,1)} s</time><span class="event-policy">${e.policy==='VECR'?'VECR':'B2'}</span><span>${escape(e.kind==='goto_accepted'?`Accepted target ${fixed(e.target_east)}, ${fixed(e.target_north)} m`:e.detail)}</span></button>`).join('');render(100);}
  function tick(now){if(!playing)return;const dt=Math.min(250,now-lastTick);lastTick=now;replayTime+=dt*Number($('#replaySpeed').value);render(replayTime);if(replayTime>=Number(slider.max))stop();else raf=requestAnimationFrame(tick);}
  play.addEventListener('click',()=>{if(playing){stop();return;}if(Number(slider.value)>=Number(slider.max))render(100);playing=true;lastTick=performance.now();play.textContent='Pause replay';raf=requestAnimationFrame(tick);});
  slider.addEventListener('input',()=>{stop();render(Number(slider.value));});$('#resetReplay').addEventListener('click',()=>{stop();render(100);});$('#replayCase').addEventListener('change',e=>loadCase(e.target.value));$('#eventList').addEventListener('click',e=>{const b=e.target.closest('button[data-time]');if(b){stop();render(Number(b.dataset.time));}});document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
  loadCase('N07');
})();
